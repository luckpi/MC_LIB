#include "pwm.h"
#include "common.h"
#include "timer3.h"
#include "gpio.h"
#include "init.h"

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
 函 数 名  : PWMChangeDuty
 功能描述  : PWM0占空比更新
 输入参数  : u16data
 输出参数  : void
*****************************************************************************/
void PWMChangeDuty(uint16_t Ta, uint16_t Tb, uint16_t Tc)
{
    Tim3_M23_CCR_Set(Tim3CCR0A, (Tc * PWM_FRE_SETATA) >> 15); //设置比较值A,(PWM互补模式下只需要设置比较值A)
    Tim3_M23_CCR_Set(Tim3CCR1A, (Tb * PWM_FRE_SETATA) >> 15);
    Tim3_M23_CCR_Set(Tim3CCR2A, (Ta * PWM_FRE_SETATA) >> 15);
}
