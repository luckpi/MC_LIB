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
 功能描述  : 采集Ia,Ib相电流
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
        //把电流转变成%比格式 2048转32768 //最大力矩 = 参考电压/(采样电阻*ADC放大倍数)
        SVM.Ia = -((uint16_t)(*(BaseJqrResultAddress)) - SVM.Ia_C) * 16;
        SVM.Ib = -((uint16_t)(*(BaseJqrResultAddress + 1)) - SVM.Ib_C) * 16;
    }
}
/*****************************************************************************
 函 数 名  : ADC_Calibrate
 功能描述  : 校准相电流
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void ADC_Calibrate(void)
{
    static uint8_t count = 0;
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
        AngleSin_Cos.IQAngle = -32768;
        // AngleSin_Cos.Angle_X = 60;
        break;
    case mcAlign:
        HoldParm.MainDetectCnt++;
        Clark_Cala(&SVM);
        Park_Cala(&SVM);
        PI_Control();
        IQSin_Cos_Cale(&AngleSin_Cos);
        SVM.Sine = AngleSin_Cos.IQSin;
        SVM.Cosine = AngleSin_Cos.IQCos;
        InvPark(&SVM);
        svgendq_calc(&SVM);
        PWMChangeDuty(&SVM);
        break;
    case mcDrag:; // 强托阶段
    case mcRun:
        Clark_Cala(&SVM);
        smc.Ialpha = SVM.Ialpha; // 需要调整电流数据Q15格式
        smc.Ibeta = SVM.Ibeta;
        smc.Valpha = SVM.Valpha; // 需要调整电压数据Q15格式
        smc.Vbeta = SVM.Vbeta;
        SMC_Position_Estimation(&smc);
        CalculateParkAngle();
        Park_Cala(&SVM);
        PI_Control();
        IQSin_Cos_Cale(&AngleSin_Cos);
        SVM.Sine = AngleSin_Cos.IQSin;
        SVM.Cosine = AngleSin_Cos.IQCos;
        InvPark(&SVM);
        svgendq_calc(&SVM);
        PWMChangeDuty(&SVM);
    default:
        break;
    }
    Adc_SQR_Start();
    Tim3_ClearIntFlag(Tim3UevIrq); // 清除中断标识位
    Gpio_WriteOutputIO(GpioPortB, GpioPin7, FALSE);
}
