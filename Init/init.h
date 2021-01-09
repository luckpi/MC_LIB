#ifndef _INIT_H
#define _INIT_H
#include "gpio.h"
#include "uart.h"
#include "sysctrl.h"
#include "timer3.h"
#include "flash.h"
#include "adc.h"

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
extern void GPIO_init(void);
extern void PWM_init(void);
extern void ADC_init(void);
extern void UART_init(void);
extern void Clk_init(void);
extern void DMA_init(void);
extern void OPA_init(void);

#endif
