#include "common.h"
#include "control.h"
#include "pwm.h"
#include "adc.h"
#include "PI.h"
#include "svgen_dq.h"
#include "smc.h"
#include "MotorConfig.h"
MotorState_T mcState;
ErrorState_T error_code;
volatile HoldControlPara_T HoldParm;
volatile ADCSamplePara_T ADCSample;
volatile PWMCatchPara_T CatchParm;
/*****************************************************************************
 函 数 名  : PowerupParaInit
 功能描述  : 上电参数初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PowerupParaInit(void)
{
    PortOutput_Config(0, 0, 0, 0, 0, 0);
    mcState = mcAhead;
    error_code = normal;
    HoldParm.RotorDirection = CW;
}
/*****************************************************************************
 函 数 名  : Common_Init
 功能描述  : 全局变量初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void Common_Init(void)
{
    HoldParm.MainDetectCnt = 0;
    HoldParm.SpeedLoopCnt = 0;
    HoldParm.SpeedTime = 0;
    HoldParm.RPM = 0;
    HoldParm.Set_RPM = First_Gear;
    HoldParm.Stall_Cnt = 0;
    ADCSample.Voltage = 0;
    ADCSample.OverCurrentCnt = 0;
    ADCSample.OverVoltageCnt = 0;
    CatchParm.Period = 0;
    CatchParm.Duty = 0;
    CatchParm.Flag_Cap_Valid = 0;
    CatchParm.PWMCnt = 0;
    CatchParm.DutyCycleUse = 0;

    //SVM_Init
    SVM.Ia = 0;     // 直接测量
    SVM.Ib = 0;     // 直接测量
    SVM.Ic = 0;     // 根据Ia + Ib + Ic =0可得
    SVM.Ia_C = 0;   // Ia校准误差
    SVM.Ib_C = 0;   // Ia校准误差
    SVM.Ialpha = 0; // Clark变换输出
    SVM.Ibeta = 0;  // Clark变换输出
    SVM.Sine = 0;   // 正弦
    SVM.Cosine = 0; // 余弦
    SVM.Id = 0;     // Park d轴
    SVM.Iq = 0;     // Park q轴
    SVM.Vd = 0;     // PI输出
    SVM.Vq = 0;     // PI输出
    SVM.Valpha = 0; // Park逆变换输出
    SVM.Vbeta = 0;  // Park逆变换输出
    SVM.Ta = 0;     // SVP输出
    SVM.Tb = 0;     // SVP输出
    SVM.Tc = 0;     // SVP输出

    //SMO_Init
    smc.Valpha = 0;
    smc.Ealpha = 0;
    smc.EalphaFinal = 0;
    smc.Zalpha = 0;
    smc.EstIalpha = 0;
    smc.Vbeta = 0;
    smc.Ebeta = 0;
    smc.EbetaFinal = 0;
    smc.Zbeta = 0;
    smc.EstIbeta = 0;
    smc.Ialpha = 0;
    smc.Ibeta = 0;
    smc.Theta = 0;
    smc.Omega = 0;
    error_code = normal;

    //Motor_CFG
    MotorCfg.OpenLoopSpeed = 0;
    MotorCfg.OpenLoopSpeedEnd = END_SPEED * 65536; // 开环转度转换为角速度 1024是为了减弱加速度
    MotorCfg.OpenLoopSpeedAdd = MotorCfg.OpenLoopSpeedEnd / PWM_FREQ / OPENLOOP_TIME;

    SMC_Init(&smc, &motorParm);
    //PID_Init
    PI_Parameters();
}
