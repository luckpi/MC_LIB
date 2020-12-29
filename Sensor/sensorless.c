#include "sensorless.h"
#include "common.h"
#include "gpio.h"
#include "pwm.h"
#include "init.h"
#include "control.h"
#include "svgen_dq.h"
#include "IQmath.h"
#include "smc.h"
#include "PI.h"
/* 基于滑动均值滤波器的电流偏移计算参数 */
#define MOVING_AVG_WINDOW_SIZE          18      // 移动平均窗口样本大小 is 2^18
#define CURRENT_OFFSET_MAX              2200    // 当前偏移最大限制
#define CURRENT_OFFSET_MIN              1800    // 当前偏移最小限制
#define CURRENT_OFFSET_INIT             2048    // 由于OPAMP偏置在VDD / 2，
#define ADC_CURRENT_SCALE               16      // 将采样值转换为Q15格式
uint32_t cumulative_sum_phaseA = (CURRENT_OFFSET_INIT << MOVING_AVG_WINDOW_SIZE);
uint32_t cumulative_sum_phaseB = (CURRENT_OFFSET_INIT << MOVING_AVG_WINDOW_SIZE);
uint32_t moving_average_phaseA = 0;
uint32_t moving_average_phaseB = 0;

/*****************************************************************************
 函 数 名  : Moving_Average_Filter
 功能描述  : 滑动均值滤波，用于计算偏移值
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void Moving_Average_Filter(uint32_t *sum, uint32_t *avg, int16_t *i)
{
    (*sum) += (*i) - (*avg);
    (*avg) = (*sum) >> MOVING_AVG_WINDOW_SIZE;

    /* 界定偏移值 */
    if ((*avg) > CURRENT_OFFSET_MAX)
    {
        (*avg) = CURRENT_OFFSET_MAX;
    }
    else if ((*avg) < CURRENT_OFFSET_MIN)
    {
        *avg = CURRENT_OFFSET_MIN;
    }
}
/*****************************************************************************
 函 数 名  : PhaseCurrentSample
 功能描述  : 采集Ia,Ib相电流
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void PhaseCurrentSample(void)
{
    volatile uint32_t *BaseJqrResultAddress = (volatile uint32_t *)&(M0P_ADC->JQRRESULT0);
    SVM.Ia = (uint16_t)(*(BaseJqrResultAddress));     // 相电流 A
    SVM.Ib = (uint16_t)(*(BaseJqrResultAddress + 1)); // 相电流 B
    if (mcState != mcAhead)
    {
        /* 实现了滑动均值滤波器以计算当前偏移量。 滑动均值滤波的窗口大小= 2 ^ MOVING_AVG_WINDOW_SIZE */
        Moving_Average_Filter(&cumulative_sum_phaseA, &moving_average_phaseA, &SVM.Ia);
        Moving_Average_Filter(&cumulative_sum_phaseB, &moving_average_phaseB, &SVM.Ib);
        SVM.Ia -= moving_average_phaseA; // 减去偏移值
        SVM.Ib -= moving_average_phaseB;
        //把电流转变成%比格式 2048转32768 //最大力矩 = 参考电压/(采样电阻*ADC放大倍数)
        SVM.Ia *= ADC_CURRENT_SCALE; // 转Q15
        SVM.Ib *= ADC_CURRENT_SCALE;
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
        cumulative_sum_phaseA = SVM.Ia_C << MOVING_AVG_WINDOW_SIZE;
        cumulative_sum_phaseB = SVM.Ib_C << MOVING_AVG_WINDOW_SIZE;
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
        AngleSin_Cos.IQAngle = 0;
        break;
    case mcAlign:
        HoldParm.MainDetectCnt++;
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
        break;
    default:
        break;
    }
    Adc_SQR_Start();
    Tim3_ClearIntFlag(Tim3UevIrq); // 清除中断标识位
    Gpio_WriteOutputIO(GpioPortB, GpioPin7, FALSE);
}
