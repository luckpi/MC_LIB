#ifndef _CONTROL_H
#define _CONTROL_H
#include "common.h"

/* Motor Config Parameter data type */
typedef struct
{
    int32_t OpenLoopSpeed;    // 开环速度
    int32_t OpenLoopSpeedEnd; // 开环结束速度
    int16_t OpenLoopSpeedAdd; // 开环速度增量
    int16_t OmegaMin;
    int16_t OmegaMax;
} MOTOR_CONFIG;
extern MOTOR_CONFIG MotorCfg;
extern void MotorControl(void);
extern void MotorStop(void);
extern void EnterRunInit(void);
extern void CalculateParkAngle(void);
#endif
