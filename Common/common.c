#include "common.h"
#include "pwm.h"
#include "adc.h"
MotorState_T mcState;
ErrorState_T error_code;
volatile HoldControlPara_T HoldParm;
volatile ADCSamplePara_T ADCSample;
volatile PWMCatchPara_T CatchParm;
volatile SensorPara_T Halless;
/*****************************************************************************
 函 数 名  : PowerupParaInit
 功能描述  : 上电参数初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PowerupParaInit(void)
{
    // PortOutput_Config(0, 0, 0, 0, 0, 0);
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
    HoldParm.PWMDutyCycle = 0;
    HoldParm.SpeedTime = 0;
    HoldParm.DragTime = 500;
    HoldParm.RPM = 0;
    HoldParm.Set_RPM = First_Gear;
    HoldParm.SpeedTime_Cnt = 0;
    HoldParm.SpeedTime_Sum = 0;
    HoldParm.SpeedTimeTemp = 0;
    HoldParm.Stall_Cnt = 0;
    ADCSample.Average = 0;
    ADCSample.Voltage = 0;
    ADCSample.OverCurrentCnt = 0;
    ADCSample.OverVoltageCnt = 0;
    ADCSample.UBemf = 0;        // 反电动势U相
    ADCSample.VBemf = 0;        // 反电动势V相
    ADCSample.WBemf = 0;        // 反电动势W相
    ADCSample.NeutralPoint = 0; // 反电动势中性点
    ADCSample.Sum = 0;
    ADCSample.Num = 0;
    ADCSample.ChlState = Voltage_Chl; // 电压电流检测
    CatchParm.Period = 0;
    CatchParm.Duty = 0;
    CatchParm.Flag_Cap_Valid = 0;
    CatchParm.PWMCnt = 0;
    CatchParm.DutyCycleUse = 0;
    error_code = normal;
    Halless.HallessState = 0;
    Halless.Phase = 3;
    Halless.LastPhase = 2;
    Halless.Delay_Time = 0;
    Halless.Zero_Flag = 0;
    Halless.Check_Count = 0;
    Halless.Filter_Times = 0;
    Halless.BackEMFFilter = 0;
    // PID_init();
}
