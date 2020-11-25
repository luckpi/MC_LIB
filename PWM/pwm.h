#ifndef _PWM_H
#define _PWM_H
#include "timer3.h"
#include "common.h"
#include "svgen_dq.h"
#define UP16LIMIT(var, max, min)               \
    {                                          \
        (var) = (var) > (max) ? (max) : (var); \
        (var) = (var) < (min) ? (min) : (var); \
    }
#define PWM_Open 0x06
#define PWM_Close 0x00
#define Open_Mode 0x01
#define Close_Mode 0x00
extern void PortOutput_Config(uint8_t U1, uint8_t U2, uint8_t V1, uint8_t V2, uint8_t W1, uint8_t W2);
extern void PWMChangeDuty(p_SVGENDQ);
#endif
