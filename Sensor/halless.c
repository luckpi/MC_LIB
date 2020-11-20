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
/*Moving Average Filter based Current Offset Calculator Parameters */
#define MOVING_AVG_WINDOW_SIZE 18 // moving average window sample size is 2^18
#define CURRENT_OFFSET_MAX 2500   // current offset max limit
#define CURRENT_OFFSET_MIN 1500   // current offset min limit
#define CURRENT_OFFSET_INIT 2048  // // as the OPAMPs are biased at VDD/2, the estimate offset value is 2048 i.e. half of 4095 which is full scale value of a 12 bit ADC.
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
        SVM.Ia = (int16_t)(-(((*(BaseJqrResultAddress + 1)) - SVM.Ib_C) << 4));
        SVM.Ib = (int16_t)(-(((*(BaseJqrResultAddress)) - SVM.Ia_C) << 4));
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
        AngleSin_Cos.IQAngle = 0;
        // AngleSin_Cos.Angle_X = 60;
        break;
    case mcAlign:
        HoldParm.MainDetectCnt++;
        Clark_Cala();
        Park_Cala();
        PI_Control();
        IQSin_Cos_Cale((p_IQSin_Cos)&AngleSin_Cos);
        SVM.Sine = AngleSin_Cos.IQSin;
        SVM.Cosine = AngleSin_Cos.IQCos;
        InvPark();
        svgendq_calc();
        PWMChangeDuty((uint16_t)SVM.Ta, (uint16_t)SVM.Tb, (uint16_t)SVM.Tc);
        break;
    case mcDrag:
        Clark_Cala();
        smc.Ialpha = SVM.Ialpha; // 需要调整电流数据Q15格式
        smc.Ibeta = SVM.Ibeta;
        smc.Valpha = SVM.Valpha; // 需要调整电压数据Q15格式
        smc.Vbeta = SVM.Vbeta;
        SMC_Position_Estimation(&smc);
        StartupDrag();
        Park_Cala();
        PI_Control();
        IQSin_Cos_Cale((p_IQSin_Cos)&AngleSin_Cos);
        SVM.Sine = AngleSin_Cos.IQSin;
        SVM.Cosine = AngleSin_Cos.IQCos;
        InvPark();
        svgendq_calc();
        PWMChangeDuty((uint16_t)SVM.Ta, (uint16_t)SVM.Tb, (uint16_t)SVM.Tc);
        break;
    case mcRun:
        break;
    default:
        break;
    }
    Adc_SQR_Start();
    Tim3_ClearIntFlag(Tim3UevIrq); // 清除中断标识位
    Gpio_WriteOutputIO(GpioPortB, GpioPin7, FALSE);
}
