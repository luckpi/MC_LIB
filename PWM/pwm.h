#ifndef _PWM_H
#define _PWM_H
#include "timer3.h"
#include "common.h"
#include "svgen_dq.h"

extern void PortOutput_Config(uint8_t U1, uint8_t U2, uint8_t V1, uint8_t V2, uint8_t W1, uint8_t W2);
extern void PWMChangeDuty(p_SVGENDQ);
#endif
