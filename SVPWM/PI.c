#include "PI.h"
#include "IQmath.h"
#include "svgen_dq.h"
#include "smc.h"
float VelRefRaw;
float DoControl_Temp1, DoControl_Temp2;
short potReading;
tCtrlParm CtrlParm;
tPIParm PIParmQ;    /* Q轴电流PI控制器的参数 */
tPIParm PIParmD;    /* D轴电流PI控制器的参数 */
tPIParm PIParmQref; /* 速度PI控制器的参数 */

#define direction 1                    // 方向
#define MAX_MOTOR_CURRENT (float)(4.4) //最大电流
void MC_APP_MC_InitPI(tPIParm *pParm)
{
    pParm->qdSum = 0;
    pParm->qOut = 0;
}

void MC_APP_MC_CalcPI(tPIParm *pParm)
{
    float Err;
    float U;
    float Exc;

    Err = pParm->qInRef - pParm->qInMeas;
    pParm->qErr = Err;
    U = pParm->qdSum + pParm->qKp * Err;

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

    pParm->qdSum = pParm->qdSum + pParm->qKi * Err - pParm->qKc * Exc;
}
inline void MC_APP_MC_DoControl(void)
{
    if (mcState == mcDrag) // 开环强拖
    {
        // OPENLOOP:  force rotating angle,Vd,Vq
        // if (mcState == mcDrag) //开环结束
        // {
        //     // just changed to openloop
        //     MC_APP_MC_CONTROL.bit.ChangeMode = 0;
        //     // synchronize angles

        //     // VqRef & VdRef not used
        //     CtrlParm.IqRef = 0;
        //     CtrlParm.IdRef = 0;

        //     // reinit vars for initial speed ramp
        //     Startup_Lock_Count = 0;
        //     Startup_Ramp_Angle_Rads_Per_Sec = 0;
        // }

        // q当前参考等于vel参考
        // 而d当前参考等于0
        // 要获得最大启动扭矩，请将q电流设置为最大可接受值
        // 值代表最大峰值

        CtrlParm.IqRef = Q_CURRENT_REF_OPENLOOP * direction; //控制方向

        // PI control for Q
        PIParmQ.qInMeas = SVM.Lq;
        PIParmQ.qInRef = CtrlParm.IqRef;
        MC_APP_MC_CalcPI(&PIParmQ);
        SVM.Vq = PIParmQ.qOut;

        // PI control for D
        PIParmD.qInMeas = SVM.Ld;
        PIParmD.qInRef = CtrlParm.IdRef;
        MC_APP_MC_CalcPI(&PIParmD);
        SVM.Vd = PIParmD.qOut;
    }
    else if (mcState == mcRun) // 闭环
    {
        // if (MC_APP_MC_CONTROL.bit.ChangeMode) //切换模式
        // {
        //     // just changed from openloop
        //     MC_APP_MC_CONTROL.bit.ChangeMode = 0;
        //     PIParmQref.qdSum = CtrlParm.IqRef;
        //     CtrlParm.VelRef = END_SPEED_RADS_PER_SEC_ELEC;
        //     PIParmD.qInRef = 0.0;
        //     CtrlParm.IdRef = 0.0;
        // }

        VelRefRaw = (float)((float)potReading * POT_ADC_COUNT_FW_SPEED_RATIO); //速度控制
        /* LPF */
        CtrlParm.VelRef = (RL_1MINUS_WCTS_VELREF * (CtrlParm.VelRef)) + (RL_WCTS_VELREF * (VelRefRaw));

        if (CtrlParm.VelRef < END_SPEED_RADS_PER_SEC_ELEC)
        {
            CtrlParm.VelRef = END_SPEED_RADS_PER_SEC_ELEC;
        }

        CtrlParm.IqRefmax = MAX_MOTOR_CURRENT;

        // 执行速度控制循环
        PIParmQref.qInMeas = smc.OmegaFltred;            // 反馈速度
        PIParmQref.qInRef = CtrlParm.VelRef * direction; // 电机参考速度和方向
        MC_APP_MC_CalcPI(&PIParmQref);
        CtrlParm.IqRef = PIParmQref.qOut;

        CtrlParm.IdRef = 0;

        // PI control for D
        PIParmD.qInMeas = SVM.Ld;        // This is in Amps
        PIParmD.qInRef = CtrlParm.IdRef; // This is in Amps
        MC_APP_MC_CalcPI(&PIParmD);
        SVM.Vd = PIParmD.qOut; // This is in %. If should be converted to volts, multiply with (DC/2)

        // dynamic d-q adjustment
        // with d component priority
        // vq=sqrt (vs^2 - vd^2)
        // limit vq maximum to the one resulting from the calculation above
        DoControl_Temp2 = PIParmD.qOut * PIParmD.qOut;
        DoControl_Temp1 = 0.98 - DoControl_Temp2;
        PIParmQ.qOutMax = sqrt(DoControl_Temp1);

        //Limit Q axis current
        if (CtrlParm.IqRef > CtrlParm.IqRefmax)
        {
            CtrlParm.IqRef = CtrlParm.IqRefmax;
        }

        // PI control for Q
        PIParmQ.qInMeas = SVM.Lq;        // This is in Amps
        PIParmQ.qInRef = CtrlParm.IqRef; // This is in Amps
        MC_APP_MC_CalcPI(&PIParmQ);
        SVM.Vq = PIParmQ.qOut; // This is in %. If should be converted to volts, multiply with (DC/2)
    }                          /* end of Closed Loop Vector Control */
}