#ifndef _MOTORCFG_H
#define _MOTORCFG_H
#include "common.h"

/*************** PWM and Control Timing Parameters ****************************/
/* Specify PWM Frequency in Hertz */
#define PWMFREQUENCY_HZ 16000
/* Specify dead time in micro seconds */
/* Specify PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define LOOPTIME_SEC 0.0000625

/****************************** Motor Parameters ******************************/
/********************  support xls file definitions begin *********************/
/* The following values are given in the xls attached file */
/* 极对数 */
#define NOPOLESPAIRS 4
/* 开环结束切闭环速度 */
#define END_SPEED_RPM 500
/* 电机额定转速（RPM）*/
#define NOMINAL_SPEED_RPM 2700
/* 电动机的最大转速（RPM */
#define MAXIMUM_SPEED_RPM 5300

#define FW_NOMINAL_SPEED_RPM 2700

/* 归一化的ls / dt值 */
#define NORM_LSDTBASE 1397

/* 标准化rs值 */
#define NORM_LSDTBASE_SCALINGFACTOR 8

#define NORM_RS 14361

#define NORM_RS_SCALINGFACTOR 1

/* End speed of open loop ramp up converted into electrical speed */
#define ENDSPEED_ELECTR END_SPEED_RPM *NOPOLESPAIRS

//***********************SMC Params*********************************************//
#define LOOPTIMEINSEC 0.0000625 // PWM Period = 1.0 / PWMFREQUENCY
#define SPEEDLOOPFREQ 1000      // 速度环频率（以赫兹为单位）。 此值必须 \
                                                              // 为整数以避免预编译器错误
#define SPEEDLOOPTIME 0.001     // 调速周期 (1.0 / SPEEDLOOPFREQ)
#define IRP_PERCALC 16          // 每个速度计算的PWM回路 (SPEEDLOOPTIME / LOOPTIMEINSEC)
#define TRANSITION_STEPS 4      // IRP_PERCALC / 4

#define SMCGAIN 0.8      // Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC 0.05 // If measured current - estimated current   \
                           // is less than MAXLINEARSMC, the slide mode \
                           // Controller will have a linear behavior    \
                           // instead of ON/OFF. Value from (0.0 to 0.9999)

#define STARTUPRAMP_THETA_OPENLOOP_SCALER 10

#define MAX_VOLTAGE_VECTOR 0.98
// Vd and Vq vector limitation variables

#define SMO_SPEED_EST_MULTIPLIER 30000 // Q15(0.9155273)

#define THETA_FILTER_CNST 7028 //Q15(0.104719 * LOOPTIME_SEC * 32768.0) // 2*pi/60*Ts*32768

//***********************End of SMC Params************************************//
#endif
