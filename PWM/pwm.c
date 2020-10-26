#include "pwm.h"
#include "common.h"
#include "timer3.h"
#include "gpio.h"
void PortOutput_Config(uint8_t U1, uint8_t U2, uint8_t V1, uint8_t V2, uint8_t W1, uint8_t W2)
{
    /*通道输出控制  
    000：强制为0                       001：强制为1  
    010：比较匹配强制为0                011：比较匹配强制为1 
    100：比较匹配时翻转                 101：比较匹配时输出一个计数周期的高电平
    110：PWM 模式1                     111 PWM 模式2
    */
    M0P_TIM3_MODE23->FLTR_f.OCMA0_FLTA0 = U1;
    M0P_TIM3_MODE23->FLTR_f.OCMB0_FLTB0 = U2;
    M0P_TIM3_MODE23->FLTR_f.OCMA1_FLTA1 = V1;
    M0P_TIM3_MODE23->FLTR_f.OCMB1_FLTB1 = V2;
    M0P_TIM3_MODE23->FLTR_f.OCMA2_FLTA2 = W1;
    M0P_TIM3_MODE23->FLTR_f.OCMB2_FLTB2 = W2;
}
/*****************************************************************************
 函 数 名  : PWMSwitchPhase
 功能描述  : PWM0换相函数[CW:AB-AC-BC-BA-CA-CB	CCW:AB-CB-CA-BA-BC-AC]
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PWMSwitchPhase(void)
{
    // PortOutput_Config(0, 1, 0, 1, 0, 1);
    if (HoldParm.RotorDirection == CW)
    {
        switch (Halless.Phase)
        {
        case 1: // U+ V-
            // PortOutput_Config(0x06, 0x01, 0x00, 0x00, 0x00, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMA2_FLTA2 = PWM_Close;
            M0P_TIM3_MODE23->FLTR_f.OCMA0_FLTA0 = PWM_Open;
            break;
        case 2: // U+ W-
            // PortOutput_Config(0x06, 0x01, 0x00, 0x01, 0x00, 0x00);
            M0P_TIM3_MODE23->FLTR_f.OCMB1_FLTB1 = Close_Mode;
            M0P_TIM3_MODE23->FLTR_f.OCMB2_FLTB2 = Open_Mode;
            break;
        case 3: // V+ W-
            // PortOutput_Config(0x00, 0x01, 0x06, 0x01, 0x00, 0x00);
            M0P_TIM3_MODE23->FLTR_f.OCMA0_FLTA0 = PWM_Close;
            M0P_TIM3_MODE23->FLTR_f.OCMA1_FLTA1 = PWM_Open;
            break;
        case 4: // V+ U-
            // PortOutput_Config(0x00, 0x00, 0x06, 0x01, 0x00, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMB2_FLTB2 = Close_Mode;
            M0P_TIM3_MODE23->FLTR_f.OCMB0_FLTB0 = Open_Mode;
            break;
        case 5: // W+ U-
            // PortOutput_Config(0x00, 0x00, 0x00, 0x01, 0x06, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMA1_FLTA1 = PWM_Close;
            M0P_TIM3_MODE23->FLTR_f.OCMA2_FLTA2 = PWM_Open;
            break;
        case 6: // W+ V-
            // PortOutput_Config(0x00, 0x01, 0x00, 0x00, 0x06, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMB0_FLTB0 = Close_Mode;
            M0P_TIM3_MODE23->FLTR_f.OCMB1_FLTB1 = Open_Mode;
            break;
        default:
            PortOutput_Config(0x06, 0x01, 0x00, 0x00, 0x00, 0x01);
            break;
        }
    }
    else if (HoldParm.RotorDirection == CCW)
    {
        switch (Halless.Phase)
        {
        case 1: // U+ V-
            // PortOutput_Config(0x06, 0x01, 0x00, 0x00, 0x00, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMB2_FLTB2 = Close_Mode;
            M0P_TIM3_MODE23->FLTR_f.OCMB1_FLTB1 = Open_Mode;
            break;
        case 2: // W+ V-
            // PortOutput_Config(0x00, 0x01, 0x0, 0x00, 0x06, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMA0_FLTA0 = PWM_Close;
            M0P_TIM3_MODE23->FLTR_f.OCMA2_FLTA2 = PWM_Open;
            break;
        case 3: // W+ U-
            // PortOutput_Config(0x00, 0x00, 0x00, 0x01, 0x06, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMB1_FLTB1 = Close_Mode;
            M0P_TIM3_MODE23->FLTR_f.OCMB0_FLTB0 = Open_Mode;
            break;
        case 4: // V+ U-
            // PortOutput_Config(0x00, 0x00, 0x06, 0x01, 0x00, 0x01);
            M0P_TIM3_MODE23->FLTR_f.OCMA2_FLTA2 = PWM_Close;
            M0P_TIM3_MODE23->FLTR_f.OCMA1_FLTA1 = PWM_Open;
            break;
        case 5: // V+ W-
            // PortOutput_Config(0x00, 0x01, 0x06, 0x01, 0x00, 0x00);
            M0P_TIM3_MODE23->FLTR_f.OCMB0_FLTB0 = Close_Mode;
            M0P_TIM3_MODE23->FLTR_f.OCMB2_FLTB2 = Open_Mode;
            break;
        case 6: // U+ W-
            // PortOutput_Config(0x06, 0x01, 0x00, 0x01, 0x00, 0x00);
            M0P_TIM3_MODE23->FLTR_f.OCMA1_FLTA1 = PWM_Close;
            M0P_TIM3_MODE23->FLTR_f.OCMA0_FLTA0 = PWM_Open;
            break;
        default:
            PortOutput_Config(0x06, 0x01, 0x00, 0x00, 0x00, 0x01);
            break;
        }
    }
    // if (Halless.Phase == 1)
    //     Gpio_WriteOutputIO(GpioPortA, GpioPin11, TRUE);
    // else
    //     Gpio_WriteOutputIO(GpioPortA, GpioPin11, FALSE);
}
/*****************************************************************************
 函 数 名  : PWMChangeDuty
 功能描述  : PWM0占空比更新
 输入参数  : u16data
 输出参数  : void
*****************************************************************************/
void PWMChangeDuty(uint16_t u16data)
{
    Tim3_M23_CCR_Set(Tim3CCR0A, u16data); //设置比较值A,(PWM互补模式下只需要设置比较值A)
    Tim3_M23_CCR_Set(Tim3CCR1A, u16data);
    Tim3_M23_CCR_Set(Tim3CCR2A, u16data);
}
