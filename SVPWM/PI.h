#ifndef _PI_H
#define _PI_H
#include "common.h"
#define PWM_FREQ 16000
#define PWM_TS (float)(1.0 / PWM_FREQ)
#define RL_WC_VELREF_FIL ((float)1.0)                       /* cross frequency of velocity reference filter */
#define RL_WCTS_VELREF (float)(RL_WC_VELREF_FIL * PWM_TS)   /* Wc*Ts */
#define RL_1MINUS_WCTS_VELREF (float)(1.0 - RL_WCTS_VELREF) /* 1 - Wc*Ts */
#define POT_ADC_COUNT_FW_SPEED_RATIO (float)(NOMINAL_SPEED_RAD_PER_SEC_ELEC / MAX_ADC_COUNT)
#define Q_CURRENT_REF_OPENLOOP ((float)0.4) /* 启动-电动机在电流控制模式下启动加速 */
typedef struct
{
    float qdSum;
    float qKp;
    float qKi;
    float qKc;
    float qOutMax;
    float qOutMin;
    float qInRef;
    float qInMeas;
    float qOut;
    float qErr;
} tPIParm;
typedef struct
{
    float VelRef;   // 参考速度
    float IdRef;    // Vd磁通参考值
    float IqRef;    // Vq转矩参考值
    float qRefRamp; // 斜坡速度参考值
    float qDiff;    // 坡道速度
    float IdRefFF;  // 弱磁期间前馈Idref
    float IdRefPI;  // 弱场Idref PI输出
    float IqRefmax; // 最大Q轴电流
} tCtrlParm;
#endif