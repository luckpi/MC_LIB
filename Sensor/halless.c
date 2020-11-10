#include "halless.h"
#include "common.h"
#include "gpio.h"
#include "pwm.h"
#include "init.h"
#include "control.h"
#include "svgen_dq.h"
#include "IQmath.h"
/*****************************************************************************
 函 数 名  : CalcSpeed
 功能描述  : 转速计算
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CalcSpeedTime(void)
{
}
/*****************************************************************************
 函 数 名  : CheckZeroCrossing
 功能描述  : 过零点检测，采用择多滤波，在PWM高电平中间进行检测
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CheckZeroCrossing(void)
{
}
/*****************************************************************************
 函 数 名  : ThreeBemfSample
 功能描述  : 三相BEMF采样
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void PhaseCurrentSample(void)
{
    volatile uint32_t *BaseJqrResultAddress = (volatile uint32_t *)&(M0P_ADC->JQRRESULT0);
    ADCSample.UBemf = (uint16_t)(*(BaseJqrResultAddress));
    ADCSample.VBemf = (uint16_t)(*(BaseJqrResultAddress + 1));
    ADCSample.WBemf = (uint16_t)(*(BaseJqrResultAddress + 2));
}
/*****************************************************************************
 函 数 名  : ADCAnalogSample
 功能描述  : 采样，电流、电压
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
// static void ADCAnalogSample(void)
// {
// }
/*****************************************************************************
 函 数 名  : ADC_ISR
 功能描述  : ADC中断
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void ADC_ISR(void)
{
    PhaseCurrentSample();
    // switch (mcState)
    // {

    // // case mcDrag:
    // //     ThreeBemfSample();
    // //     // ADCAnalogSample();
    // //     CheckZeroCrossing();
    // //     StartupDrag();
    // //     break;
    // // case mcRun:
    // //     HoldParm.SpeedLoopCnt++;
    // //     ThreeBemfSample();
    // //     // ADCAnalogSample();
    // //     CheckZeroCrossing();
    // //     break;
    // // default:
    // //     break;
    // }
    Adc_SQR_Start();
    Tim3_ClearIntFlag(Tim3UevIrq); // 清除中断标识位
}
