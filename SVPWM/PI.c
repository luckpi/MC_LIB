#include "PI.h"
#include "smc.h"
#include "IQmath.h"
#include "fdweak.h"
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
 函 数 名  : PI_Init
 功能描述  : 初始化PI参数
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PI_Init(void)
{
    // 速度环和开环参数初始化
    CtrlParm.qDiff = 0;
    CtrlParm.trans_counter = 0;
    CtrlParm.OpenLoopSpeed = 0;
    CtrlParm.SpeedRampCount = 0;
    CtrlParm.SpeedLoop_FLAG = 0;
    CtrlParm.RotorDirection = CW;
    CtrlParm.IqRefmax = MAX_VOLTAGE_VECTOR;
    CtrlParm.OpenLoopSpeedEnd = END_SPEED * 65536; // 单位周期角速度增益 (END_SPEED << THETA_OPENLOOP_SCALER)
    CtrlParm.OpenLoopSpeedAdd = CtrlParm.OpenLoopSpeedEnd / PWM_FREQ / OPENLOOP_TIME;  // 单位周期增量
    CtrlParm.OmegaMin = END_SPEED_RPM * NOPOLESPAIRS * 65535.0 / 60.0 / SPEEDLOOPFREQ; // RPM转Omega
    CtrlParm.OmegaMax = NOMINAL_SPEED_RPM * NOPOLESPAIRS * 65535.0 / 60.0 / SPEEDLOOPFREQ;

    // PI D Term
    PIParmD.qKp = Q15(D_CURRCNTR_PTERM) * NKo;
    PIParmD.qKi = Q15(D_CURRCNTR_ITERM);
    PIParmD.qKc = Q15(D_CURRCNTR_CTERM);
    PIParmD.qOutMax = Q15(D_CURRCNTR_OUTMAX);
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

    // PI Q Term
    PIParmQ.qKp = Q15(Q_CURRCNTR_PTERM) * NKo;
    PIParmQ.qKi = Q15(Q_CURRCNTR_ITERM);
    PIParmQ.qKc = Q15(Q_CURRCNTR_CTERM);
    PIParmQ.qOutMax = Q15(Q_CURRCNTR_OUTMAX);
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

    // PI Qref Term
    PIParmQref.qKp = Q15(SPEEDCNTR_PTERM) * NKo;
    PIParmQref.qKi = Q15(SPEEDCNTR_ITERM);
    PIParmQref.qKc = Q15(SPEEDCNTR_CTERM);
    PIParmQref.qOutMax = Q15(SPEEDCNTR_OUTMAX);
    PIParmQref.qOutMin = -PIParmQref.qOutMax;

    InitPI(&PIParmQref);

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
    int32_t U;
    int32_t Exc;

    pParm->qErr = pParm->qInRef - pParm->qInMeas;
    U = pParm->qdSum + _IQmpy(pParm->qKp, pParm->qErr);

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
    pParm->qdSum += _IQmpy(pParm->qKi, pParm->qErr) - _IQmpy(pParm->qKc, Exc);
}
/*****************************************************************************
 函 数 名  : PI_Control
 功能描述  : PI控制器
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PI_Control(void)
{
    volatile int32_t temp, VelRefRaw;
    // 弱磁模式开关
#ifdef FDWEAK_MODE
    CtrlParm.IdRef = FieldWeakening(Abs(CtrlParm.VelRef));
#else
    CtrlParm.IdRef = 0;
#endif
    // D轴PI控制
    PIParmD.qInMeas = svm.Id;
    PIParmD.qInRef = CtrlParm.IdRef;
    CalcPI(&PIParmD);
    svm.Vd = PIParmD.qOut; // 这是%，如果应转换为V，则乘以 (DC / 2)

    if (CtrlParm.SpeedLoop_FLAG == 1)
    {
        if (++CtrlParm.SpeedRampCount >= SPEEDREFRAMP_COUNT)
        {
            VelRefRaw = (((ADCSample.POT - 2000) * (CtrlParm.OmegaMax - CtrlParm.OmegaMin)) >> 11) + CtrlParm.OmegaMin; //速度电位器调节
            if (VelRefRaw < CtrlParm.OmegaMin)
            {
                VelRefRaw = CtrlParm.OmegaMin;
            }
            else if (VelRefRaw > CtrlParm.OmegaMax)
            {
                VelRefRaw = CtrlParm.OmegaMax;
            }
            CtrlParm.qDiff = CtrlParm.VelRef - VelRefRaw;
            if (CtrlParm.qDiff < -SPEEDREFRAMP)
            {
                CtrlParm.VelRef += SPEEDREFRAMP;
            }
            else if (CtrlParm.qDiff > SPEEDREFRAMP)
            {

                CtrlParm.VelRef -= SPEEDREFRAMP;
            }
            else
            {
                CtrlParm.VelRef = VelRefRaw;
            }
            CtrlParm.SpeedRampCount = 0;
        }
#ifdef TORQUE_MODE
        CtrlParm.IqRef = CtrlParm.VelRef;
#else
        PIParmQref.qInMeas = smc.OmegaFltred;                          // 反馈速度
        PIParmQref.qInRef = CtrlParm.VelRef * CtrlParm.RotorDirection; // 电机参考速度和方向
        CalcPI(&PIParmQref);
        CtrlParm.IqRef = PIParmQref.qOut;
#endif
        // 根据Vd动态限幅Vq，Vs <= 95%
        // Vs = sqrt(Vd^2 + Vq^2) < 0.95 -> Vq = sqrt(0.95^2 - Vd^2)
        temp = svm.Vd * svm.Vd;
        temp = (MAX_VOLTAGE_VECTOR << 15) - temp;
        PIParmQ.qOutMax = IQSqrt(temp);
        PIParmQ.qOutMin = -PIParmQ.qOutMax;
    }
    else
    {
        CtrlParm.IqRef = Q_CURRENT_REF_OPENLOOP * CtrlParm.RotorDirection;
    }

    // Q轴限幅
    if (Abs(CtrlParm.IqRef) > CtrlParm.IqRefmax)
    {
        CtrlParm.IqRef = CtrlParm.IqRefmax * CtrlParm.RotorDirection;
    }

    // Q轴PI控制
    PIParmQ.qInMeas = svm.Iq;
    PIParmQ.qInRef = CtrlParm.IqRef;
    CalcPI(&PIParmQ);
    svm.Vq = PIParmQ.qOut; // 这是%，如果应转换为V，则乘以 (DC / 2)
}
