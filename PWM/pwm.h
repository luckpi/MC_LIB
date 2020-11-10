#ifndef _PWM_H
#define _PWM_H
#include "timer3.h"
#define UP16LIMIT(var, max, min)               \
    {                                          \
        (var) = (var) > (max) ? (max) : (var); \
        (var) = (var) < (min) ? (min) : (var); \
    }
#define PWM_Open 0x06
#define PWM_Close 0x00
#define Open_Mode 0x01
#define Close_Mode 0x00
extern void PWMSwitchPhase(void);
extern void PWMChangeDuty(uint16_t Ta, uint16_t Tb, uint16_t Tc);
#endif
