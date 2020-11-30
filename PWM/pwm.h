#ifndef _PWM_H
#define _PWM_H
#include "timer3.h"
#include "common.h"
#include "svgen_dq.h"
//  PWM配置
#define PWMSYSCLK                   (48000000)  // PWM时钟
#define SYSCLK                      (48000000)  // 除PWM外的模块时钟
#define PWM_FRE                     (16000)     //PWM的频率
#define PWM_FRE_SETATA              (1500)      //自动重装载值((PWMSYSCLK / PWM_FRE) >> 1)

extern void PortOutput_Config(uint8_t U1, uint8_t U2, uint8_t V1, uint8_t V2, uint8_t W1, uint8_t W2);
extern void PWMChangeDuty(p_SVGENDQ);
#endif
