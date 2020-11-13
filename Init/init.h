#ifndef _INIT_H
#define _INIT_H
#include "gpio.h"
#include "uart.h"
#include "sysctrl.h"
#include "timer3.h"
#include "flash.h"
#include "adc.h"
#define PWMSYSCLK (48000000) // PWM时钟
#define SYSCLK (48000000)    // 除PWM外的模块时钟

#define PWM_FRE (16000) //PWM的频率
#define PWM_FRE_SETATA ((PWMSYSCLK / PWM_FRE) >> 1)
#define PWM_DUTYCYCLE_95 ((PWM_FRE_SETATA / 20) * 19) // PWM的占空比  95%
#define PWM_DUTYCYCLE_75 ((PWM_FRE_SETATA / 4) * 3)   // PWM的占空比  75%
#define PWM_DUTYCYCLE_50 (PWM_FRE_SETATA / 2)         // PWM的占空比  50%
#define PWM_DUTYCYCLE_25 (PWM_FRE_SETATA / 4)         // PWM的占空比  25%
#define PWM_DUTYCYCLE_20 (PWM_FRE_SETATA / 5)         // PWM的占空比  20%
#define PWM_DUTYCYCLE_15 ((PWM_FRE_SETATA / 20) * 3)  // PWM的占空比  15%
#define PWM_DUTYCYCLE_10 (PWM_FRE_SETATA / 10)        // PWM的占空比  10%
#define PWM_DUTYCYCLE_05 (PWM_FRE_SETATA / 20)        // PWM的占空比  5%
#define PWM_DUTYCYCLE_00 (1)                          // PWM的占空比  0%

#define PWM_START_DUTY (PWM_DUTYCYCLE_10)
#define PWM_MIN_DUTY (PWM_DUTYCYCLE_05) // 最小占空比
#define PWM_MAX_DUTY PWM_FRE_SETATA     // 最大占空比
#define OP0_INP()                   \
    {                               \
        M0P_GPIO->PCADS_f.PC06 = 1; \
        M0P_GPIO->PCDIR_f.PC06 = 1; \
    }
#define OP0_INN()                   \
    {                               \
        M0P_GPIO->PBADS_f.PB15 = 1; \
        M0P_GPIO->PBDIR_f.PB15 = 1; \
    }
#define OP0_OUT()                   \
    {                               \
        M0P_GPIO->PCADS_f.PC07 = 1; \
        M0P_GPIO->PCDIR_f.PC07 = 1; \
    }

#define OP1_INP()                   \
    {                               \
        M0P_GPIO->PBADS_f.PB13 = 1; \
        M0P_GPIO->PBDIR_f.PB13 = 1; \
    }
#define OP1_INN()                   \
    {                               \
        M0P_GPIO->PBADS_f.PB12 = 1; \
        M0P_GPIO->PBDIR_f.PB12 = 1; \
    }
#define OP1_OUT()                   \
    {                               \
        M0P_GPIO->PBADS_f.PB14 = 1; \
        M0P_GPIO->PBDIR_f.PB14 = 1; \
    }

#define OP2_INP()                   \
    {                               \
        M0P_GPIO->PBADS_f.PB10 = 1; \
        M0P_GPIO->PBDIR_f.PB10 = 1; \
    }
#define OP2_INN()                   \
    {                               \
        M0P_GPIO->PBADS_f.PB02 = 1; \
        M0P_GPIO->PBDIR_f.PB02 = 1; \
    }
#define OP2_OUT()                   \
    {                               \
        M0P_GPIO->PBADS_f.PB11 = 1; \
        M0P_GPIO->PBDIR_f.PB11 = 1; \
    }
extern void LED_Init(void);
extern void PWM_Init(void);
extern void ADC_Init(void);
extern void UART_Init(void);
extern void Clk_Init(void);
extern void DMA_Init(void);
extern void OPA_GpMode_Test(void);
#endif
