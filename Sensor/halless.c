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
 函 数 名  : PhaseCurrentSample
 功能描述  : 相电流采样
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void PhaseCurrentSample(void)
{
    volatile uint32_t *BaseJqrResultAddress = (volatile uint32_t *)&(M0P_ADC->JQRRESULT0);
    if (mcState == mcAhead)
    {
        SVP.Ia = (uint16_t)(*(BaseJqrResultAddress));
        SVP.Ib = (uint16_t)(*(BaseJqrResultAddress + 1));
    }
    else
    {
        SVP.Ia = (uint16_t)(*(BaseJqrResultAddress)) - SVP.Ia_C;
        SVP.Ib = (uint16_t)(*(BaseJqrResultAddress + 1)) - SVP.Ib_C;
    }
}
/*****************************************************************************
 函 数 名  : PhaseCurrentSample
 功能描述  : 相电流采样
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void ADC_Calibrate(void)
{
    static uint8_t count = 0;
    PhaseCurrentSample();
    if (++count > 64)
    {
        count = 0;
        SVP.Ia_C >>= 6;
        SVP.Ib_C >>= 6;
        mcState = mcInit;
    }
    else
    {
        SVP.Ia_C += SVP.Ia;
        SVP.Ib_C += SVP.Ib;
    }
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
    switch (mcState)
    {
    case mcAhead:
        ADC_Calibrate();
        break;
    case mcDrag:
        // ADCAnalogSample();
        // CheckZeroCrossing();
        PhaseCurrentSample();
        StartupDrag();
        break;
    // case mcRun:
    //     HoldParm.SpeedLoopCnt++;
    //     // ADCAnalogSample();
    //     CheckZeroCrossing();
    //     break;
    default:
        break;
    }
    Adc_SQR_Start();
    Tim3_ClearIntFlag(Tim3UevIrq); // 清除中断标识位
}
