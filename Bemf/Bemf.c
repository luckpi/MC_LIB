#include "Bemf.h"
#include "common.h"
#include "gpio.h"
#include "pwm.h"
#include "init.h"
#include "control.h"
/*AND & OR 用于屏蔽有效BEMF信号的运算符*/
/*与运算只获取当前要检测反电动势的状态， 通过异或检测当前反电动势变化情况*/
const uint8_t ADC_MASK[2][8] = {0x00, 0x04, 0x02, 0x01, 0x04, 0x02, 0x01, 0x00,  //正转
                                0x00, 0x04, 0x01, 0x02, 0x04, 0x01, 0x02, 0x00}; //反转
const uint8_t ADC_XOR[2][8] = {0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,   //正转
                               0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00};  //反转
/*BEMF 择多函数滤波*/
/*通过检测多次，当捕获到三个反电动势边沿变化中，有两个有效信号则滤波完成*/
const uint8_t ADC_BEMF_FILTER[64] =
    // 0    1    2    3      4     5     6     7     8     9     10    11    12    13    14    15
    {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E,
     0x20, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E, 0x01, 0x01, 0x01, 0x36, 0x01, 0x3A, 0x3C, 0x3E,
     0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x01, 0x01, 0x01, 0x16, 0x01, 0x1A, 0x1C, 0x1E,
     0x01, 0x01, 0x01, 0x26, 0x01, 0x2A, 0x2C, 0x2E, 0x01, 0x01, 0x01, 0x36, 0x01, 0x3A, 0x3C, 0x3E};
/*****************************************************************************
 函 数 名  : CalcSpeed
 功能描述  : 转速计算
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CalcSpeedTime(void)
{
    Halless.Zero_Flag = 0;
    HoldParm.SpeedTime_Sum += HoldParm.SpeedTimeTemp;
    if (++HoldParm.SpeedTime_Cnt >= 8)
    {
        HoldParm.SpeedTime_Cnt = 0;
        HoldParm.SpeedTime = HoldParm.SpeedTime_Sum >> 3;
        HoldParm.SpeedTime_Sum = 0;
    }
    if (HoldParm.SpeedTime)
    {
        HoldParm.RPM = 200000 / (HoldParm.SpeedTime * POLE_PAIRS);
    }
    else
    {
        HoldParm.RPM = 200000 / ((HoldParm.SpeedTimeTemp + 1) * POLE_PAIRS);
    }
}
/*****************************************************************************
 函 数 名  : CheckZeroCrossing
 功能描述  : 过零点检测，采用择多滤波，在PWM高电平中间进行检测
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CheckZeroCrossing(void)
{
    // Gpio_WriteOutputIO(GpioPortA, GpioPin11, TRUE);
    if (++Halless.Check_Count >= Halless.Filter_Times)
    {
        Halless.HallessState = 0;
        ADCSample.NeutralPoint = ADCSample.UBemf + ADCSample.VBemf + ADCSample.WBemf; // 反电动势虚拟中心点
        Halless.HallessState = (Q_Calc(ADCSample.WBemf, ADCSample.NeutralPoint) << 2) | (Q_Calc(ADCSample.VBemf, ADCSample.NeutralPoint) << 1) | (Q_Calc(ADCSample.UBemf, ADCSample.NeutralPoint));
        if ((Halless.HallessState ^ ADC_XOR[HoldParm.RotorDirection][Halless.Phase]) & ADC_MASK[HoldParm.RotorDirection][Halless.Phase]) // 获取当前要检测反电动势变化
        {
            Halless.BackEMFFilter |= 0x01;
        }
        Halless.BackEMFFilter = ADC_BEMF_FILTER[Halless.BackEMFFilter]; // 择多函数滤波
        if (Halless.BackEMFFilter & 0x01)
        {
            if (++Halless.Phase > 6)
            {
                Halless.Phase = 1;
            }
            Halless.Zero_Flag = 1;
            Halless.Check_Count = 0;
            Halless.BackEMFFilter = 0;
            // if (mcState == mcRun)
            // CalcSpeedTime();
            PWMSwitchPhase(); // 换相
        }
        else if (Halless.Check_Count >= 1000 && mcState == mcRun)
        {
            MotorStop();
            mcState = mcStop;
        }
    }
    Tim3_ClearIntFlag(Tim3UevIrq); // 清除中断标识位
    // Gpio_WriteOutputIO(GpioPortA, GpioPin11, FALSE);
}
/*****************************************************************************
 函 数 名  : ThreeBemfSample
 功能描述  : 三相BEMF采样
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void ThreeBemfSample(void)
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
//     if (ADCSample.ChlState == CHL_UBemf)
//     {
//         Start_ADCSample(CHL_VBUS);
//         ADCSample.Voltage = Get_CHL_Value();
//     }
//     else
//     {
//         Start_ADCSample(CHL_IBUS);
//         ADCSample.Current = Get_CHL_Value();
//         ADCSample.Sum += ADCSample.Current;
//         if (++ADCSample.Num == 8)
//         {
//             ADCSample.Average = (ADCSample.Sum >> 3);
//             ADCSample.Sum = 0;
//             ADCSample.Num = 0;
//         }
//     }
//     Switch_ADC_CHL(ADCSample.ChlState);
// }
/*****************************************************************************
 函 数 名  : ADC_ISR
 功能描述  : ADC中断
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void ADC_ISR(void)
{
    switch (mcState)
    {
    case mcDrag:
        ThreeBemfSample();
        // ADCAnalogSample();
        StartupDrag();
        break;
    case mcRun:
        HoldParm.SpeedLoopCnt++;
        ThreeBemfSample();
        // ADCAnalogSample();
        CheckZeroCrossing();
        break;
    default:
        break;
    }
}
