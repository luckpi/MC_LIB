#include "PI.h"
#include "smc.h"
#include "IQmath.h"
#include "control.h"
#include "svgen_dq.h"
#include "MotorConfig.h"

tCtrlParm CtrlParm;
tPIParm PIParmQ;    /* Q轴电流PI控制器的参数 */
tPIParm PIParmD;    /* D轴电流PI控制器的参数 */
tPIParm PIParmQref; /* 速度PI控制器的参数 */

static void InitPI(tPIParm *pParm)
{
    pParm->qdSum = 0;
    pParm->qOut = 0;
}
/*****************************************************************************
 函 数 名  : PI_Parameters
 功能描述  : 初始化PI参数
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PI_Parameters(void)
{

    // PI D Term
    PIParmD.qKp = Q15(D_CURRCNTR_PTERM) << 4;
    PIParmD.qKi = Q15(D_CURRCNTR_ITERM);
    PIParmD.qKc = Q15(D_CURRCNTR_CTERM);
    PIParmD.qOutMax = Q15(D_CURRCNTR_OUTMAX);
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

    // PI Q Term
    PIParmQ.qKp = Q15(Q_CURRCNTR_PTERM) << 4;
    PIParmQ.qKi = Q15(Q_CURRCNTR_ITERM);
    PIParmQ.qKc = Q15(Q_CURRCNTR_CTERM);
    PIParmQ.qOutMax = Q15(Q_CURRCNTR_OUTMAX);
    PIParmQ.qOutMin = -PIParmQref.qOutMax;

    InitPI(&PIParmQ);

    // PI Qref Term
    PIParmQref.qKp = Q15(SPEEDCNTR_PTERM) << 4;
    PIParmQref.qKi = Q15(SPEEDCNTR_ITERM);
    PIParmQref.qKc = Q15(SPEEDCNTR_CTERM);
    PIParmQref.qOutMax = Q15(SPEEDCNTR_OUTMAX);
    PIParmQref.qOutMin = -PIParmQref.qOutMax;

    InitPI(&PIParmQref);

    CtrlParm.IdRef = 0; // d轴不做功

    return;
}
/*****************************************************************************
 函 数 名  : CalcPI
 功能描述  : PI计算
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void CalcPI(tPIParm *pParm)
{
    int16_t Err;
    int32_t U;
    int32_t Exc;

    Err = pParm->qInRef - pParm->qInMeas;
    pParm->qErr = Err;
    U = pParm->qdSum + _IQmpy(pParm->qKp, Err);

    if (U > pParm->qOutMax)
    {
        pParm->qOut = pParm->qOutMax;
    }
    else if (U < pParm->qOutMin)
    {
        pParm->qOut = pParm->qOutMin;
    }
    else
    {
        pParm->qOut = U;
    }

    Exc = U - pParm->qOut;
    pParm->qdSum += _IQmpy(pParm->qKi, Err) - _IQmpy(pParm->qKc, Exc);
}
/*****************************************************************************
 函 数 名  : PI_Control
 功能描述  : PI控制器
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PI_Control(void)
{
    volatile int16_t temp1, temp2, VelRefRaw;

    if (mcState == mcAlign || mcState == mcDrag) // 开环强拖
    {
        // q当前参考等于vel参考
        // 而d当前参考等于0
        // 要获得最大启动扭矩，请将q电流设置为最大可接受值

        CtrlParm.IqRef = Q_CURRENT_REF_OPENLOOP * HoldParm.RotorDirection; //控制方向

        // PI control for Q
        PIParmQ.qInMeas = SVM.Iq;
        PIParmQ.qInRef = CtrlParm.IqRef;
        CalcPI(&PIParmQ);
        SVM.Vq = PIParmQ.qOut;

        // PI control for D
        PIParmD.qInMeas = SVM.Id;
        PIParmD.qInRef = CtrlParm.IdRef;
        CalcPI(&PIParmD);
        SVM.Vd = PIParmD.qOut;
    }
    else if (mcState == mcRun) // 闭环
    {
        if (AccumThetaCnt == 0)
        {
            VelRefRaw = ((ADCSample.POT - 1900) * MotorCfg.OmegaMax) >> 11; //速度控制，值瞎给的
            // VelRefRaw = MotorCfg.OmegaMin;
            // 执行速度控制循环
            if (VelRefRaw < MotorCfg.OmegaMin)
            {
                VelRefRaw = MotorCfg.OmegaMin;
            }
            else if (VelRefRaw > MotorCfg.OmegaMax)
            {
                VelRefRaw = MotorCfg.OmegaMax;
            }
            temp2 = CtrlParm.VelRef - VelRefRaw;
            if (temp2 < 0)
            {
                CtrlParm.VelRef += SPEEDREFRAMP;
            }
            else if (temp2 > 0)
            {
                if (temp2 > SPEEDREFRAMP)
                {
                    CtrlParm.VelRef -= SPEEDREFRAMP;
                }
                else
                {
                    CtrlParm.VelRef = VelRefRaw;
                }
            }
            PIParmQref.qInMeas = smc.Omega;                          // 反馈速度
            PIParmQref.qInRef = CtrlParm.VelRef * HoldParm.RotorDirection; // 电机参考速度和方向
            CalcPI(&PIParmQref);
            CtrlParm.IqRef = PIParmQref.qOut;
        }
        CtrlParm.IdRef = 0;
        // PI control for D
        PIParmD.qInMeas = SVM.Id;
        PIParmD.qInRef = CtrlParm.IdRef;
        CalcPI(&PIParmD);
        SVM.Vd = PIParmD.qOut; // 这是%，如果应转换为V，则乘以 (DC / 2)
        /* 具有d分量优先级的动态d-q调整*/
        // 向量限制
        // Vd is 不限
        // Vq is 限制，因此向量Vs小于最大值 95%.
        // Vs = SQRT(Vd^2 + Vq^2) < 0.95
        // Vq = SQRT(0.95^2 - Vd^2)
        // temp1 = (int16_t)(_IQmpy(PIParmD.qOut, PIParmD.qOut));
        // temp1 = MAX_VOLTAGE_VECTOR - temp1;
        // PIParmQ.qOutMax = IQSqrt(temp1 << 15);
        // PIParmQ.qOutMin = -PIParmQ.qOutMax;

        CtrlParm.IqRefmax = Q_MAX;

        //Limit Q axis current
        if (CtrlParm.IqRef > CtrlParm.IqRefmax)
        {
            CtrlParm.IqRef = CtrlParm.IqRefmax;
        }

        // PI control for Q
        PIParmQ.qInMeas = SVM.Iq;
        PIParmQ.qInRef = CtrlParm.IqRef;
        CalcPI(&PIParmQ);
        SVM.Vq = PIParmQ.qOut; // 这是%，如果应转换为V，则乘以 (DC / 2)
    }
}
