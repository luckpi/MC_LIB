#include "PI.h"
#include "smc.h"
#include "atan2.h"
#include "IQmath.h"
#include "control.h"
#include "svgen_dq.h"
#include "MotorConfig.h"

// float VelRefRaw;
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
    PIParmD.qKp = D_CURRCNTR_PTERM;
    PIParmD.qKi = D_CURRCNTR_ITERM;
    PIParmD.qKc = D_CURRCNTR_CTERM;
    PIParmD.qOutMax = D_CURRCNTR_OUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

    // PI Q Term
    PIParmQ.qKp = Q_CURRCNTR_PTERM;
    PIParmQ.qKi = Q_CURRCNTR_ITERM;
    PIParmQ.qKc = Q_CURRCNTR_CTERM;
    PIParmQ.qOutMax = Q_CURRCNTR_OUTMAX;
    PIParmQ.qOutMin = -PIParmQref.qOutMax;

    InitPI(&PIParmQ);

    // PI Qref Term
    PIParmQref.qKp = SPEEDCNTR_PTERM;
    PIParmQref.qKi = SPEEDCNTR_ITERM;
    PIParmQref.qKc = SPEEDCNTR_CTERM;
    PIParmQref.qOutMax = SPEEDCNTR_OUTMAX;
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
    int32_t Err;
    int32_t U;
    int32_t Exc;

    Err = pParm->qInRef - pParm->qInMeas;
    pParm->qErr = Err;
    U = pParm->qdSum + _IQmpy((pParm->qKp << 4), Err);

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
    pParm->qdSum += (int16_t)(_IQmpy(pParm->qKi, Err)) - (int16_t)(_IQmpy(pParm->qKc, Exc));
}
/*****************************************************************************
 函 数 名  : PI_Control
 功能描述  : PI控制器
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PI_Control(void)
{
    volatile int16_t temp1;
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

        // VelRefRaw = (float)(ADCSample.POT * POT_ADC_COUNT_FW_SPEED_RATIO); //速度控制，值瞎给的
        // /* LPF */
        // CtrlParm.VelRef = (RL_1MINUS_WCTS_VELREF * (CtrlParm.VelRef)) + (RL_WCTS_VELREF * (VelRefRaw));
        // CtrlParm.VelRef = (ADCSample.POT * POT_ADC_COUNT_FW_SPEED_RATIO) << 11;
        if (CtrlParm.VelRef < MotorCfg.OpenLoopSpeedEnd)
        {
            CtrlParm.VelRef = MotorCfg.OpenLoopSpeedEnd;
        }

        // 执行速度控制循环
        PIParmQref.qInMeas = smc.OmegaFltred;                          // 反馈速度
        PIParmQref.qInRef = CtrlParm.VelRef * HoldParm.RotorDirection; // 电机参考速度和方向
        CalcPI(&PIParmQref);
        CtrlParm.IqRef = PIParmQref.qOut;

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
