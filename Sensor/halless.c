#include "halless.h"
#include "common.h"
#include "gpio.h"
#include "pwm.h"
#include "init.h"
#include "control.h"
#include "svgen_dq.h"
#include "IQmath.h"
#include "smc.h"
#include "PI.h"

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
        SVM.Ia = (uint16_t)(*(BaseJqrResultAddress));
        SVM.Ib = (uint16_t)(*(BaseJqrResultAddress + 1));
    }
    else
    {
        SVM.Ia = (uint16_t)(*(BaseJqrResultAddress)) - SVM.Ia_C;
        SVM.Ib = (uint16_t)(*(BaseJqrResultAddress + 1)) - SVM.Ib_C;
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
        SVM.Ia_C >>= 6;
        SVM.Ib_C >>= 6;
        mcState = mcInit;
    }
    else
    {
        SVM.Ia_C += SVM.Ia;
        SVM.Ib_C += SVM.Ib;
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
    Gpio_WriteOutputIO(GpioPortB, GpioPin7, TRUE);
    PhaseCurrentSample();
    switch (mcState)
    {
    case mcAhead:
        ADC_Calibrate();
        SMCInit(&smc);
        AngleSin_Cos.IQAngle = 32767;
        break;
    case mcDrag:
        // ADCAnalogSample();
        // CheckZeroCrossing();
        PhaseCurrentSample();
        Clark_Cala();
        smc.Ibeta = SVM.Ibeta << 4; // 需要调整电流数据格式
        smc.Ialpha = SVM.Ialpha << 4;
        smc.Valpha = (SVM.Valpha * smc.MaxVoltage) >> 9; // 需要调整电压数据格式
        smc.Vbeta = (SVM.Vbeta * smc.MaxVoltage) >> 9;
        IQSin_Cos_Cale((p_IQSin_Cos)&AngleSin_Cos);
        SVM.Sine = AngleSin_Cos.IQSin;
        SVM.Cosine = AngleSin_Cos.IQCos;
        Park_Cala();
        SMC_Position_Estimation_Inline(&smc);
        StartupDrag();
        InvPark();
        svgendq_calc();
        PWMChangeDuty((uint16_t)SVM.Ta, (uint16_t)SVM.Tb, (uint16_t)SVM.Tc);
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
    Gpio_WriteOutputIO(GpioPortB, GpioPin7, FALSE);
}
