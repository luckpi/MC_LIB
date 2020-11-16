#ifndef _ATAN2_H
#define _ATAN2_H
#include "common.h"
#define M_PI 32768
#define M_PI_2 16384
#define M_PI_4 8192

#define RL_A 2544
#define RL_B -9419
#define RL_C (M_PI_4 - RL_A - RL_B)
#define RL_EPSILON_F 1 /* small float value for zero-check */
int16_t Atan2(int16_t y, int16_t x);
#endif
