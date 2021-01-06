#include "PI.h"
#include "adc.h"
#include "pwm.h"
#include "smc.h"
#include "common.h"
#include "fdweak.h"
#include "control.h"
#include "svgen_dq.h"
#include "MotorConfig.h"

MotorState_T mcState;
ErrorState_T error_code;
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
}
/*****************************************************************************
 函 数 名  : Common_Init
 功能描述  : 全局变量初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void Common_Init(void)
{
    ADCSample.Voltage = 0;
    ADCSample.POT = 0;
    CatchParm.Period = 0;
    CatchParm.Duty = 0;
    CatchParm.Flag_Cap_Valid = 0;
    CatchParm.PWMCnt = 0;
    CatchParm.DutyCycleUse = 0;
    error_code = normal;

    // SVPWM参数初始化
    svgendq_Init(&svm);

    // 滑模控制器初始化
    SMC_Init(&smc, &motorParm);

    // PI初始化
    PI_Init();

#ifdef FDWEAK_MODE
    // 弱磁初始化
    FWInit();
#endif
}
