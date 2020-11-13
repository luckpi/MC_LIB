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

/* 标准电流常量 */
#define NORM_CURRENT_CONST 0.000134 // = 峰值电流 / 2^15

/* 归一化的ls / dt值 */
#define NORM_LSDTBASE 1397

/* 标准化rs值 */
#define NORM_LSDTBASE_SCALINGFACTOR 8

#define NORM_RS 14361

#define NORM_RS_SCALINGFACTOR 1

/**********************  support xls file definitions end *********************/

/* current transformation macro, used below */
#define NORM_CURRENT(current_real) (Q15(current_real / NORM_CURRENT_CONST / 32768))

/* Open loop startup constants */

/* The following values depends on the PWM frequency,
 lock time is the time needed for motor's poles alignment 
before the open loop speed ramp up */
/* This number is: 20,000 is 1 second. */
#define LOCK_TIME 4000
/* Open loop acceleration */
#define OPENLOOP_RAMPSPEED_INCREASERATE 10
/* Open loop q current setup - */
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(1.41)

/* Maximum motor speed converted into electrical speed */
#define MAXIMUMSPEED_ELECTR MAXIMUM_SPEED_RPM *NOPOLESPAIRS
/* Nominal motor speed converted into electrical speed */
#define NOMINALSPEED_ELECTR NOMINAL_SPEED_RPM *NOPOLESPAIRS

/* End speed converted to fit the startup ramp */
#define END_SPEED (END_SPEED_RPM * NOPOLESPAIRS * LOOPTIME_SEC * 65536 / 60.0) * 1024
/* End speed of open loop ramp up converted into electrical speed */
#define ENDSPEED_ELECTR END_SPEED_RPM *NOPOLESPAIRS

/* In case of the potentiometer speed reference, a reference ramp
is needed for assuring the motor can follow the reference imposed /
minimum value accepted */
#define SPEEDREFRAMP Q15(0.00003)

/* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
#define SPEEDREFRAMP_COUNT 3
/* PI controllers tuning values - */
/* D Control Loop Coefficients */
#define D_CURRCNTR_PTERM Q15(0.02)
#define D_CURRCNTR_ITERM Q15(0.002)
#define D_CURRCNTR_CTERM Q15(0.999)
#define D_CURRCNTR_OUTMAX 0x7FFF



/* Q Control Loop Coefficients */
#define Q_CURRCNTR_PTERM Q15(0.02)
#define Q_CURRCNTR_ITERM Q15(0.002)
#define Q_CURRCNTR_CTERM Q15(0.999)
#define Q_CURRCNTR_OUTMAX 0x7FFF

/* Velocity Control Loop Coefficients */
#define SPEEDCNTR_PTERM Q15(0.5)
#define SPEEDCNTR_ITERM Q15(0.005)
#define SPEEDCNTR_CTERM Q15(0.999)
#define SPEEDCNTR_OUTMAX 0x5000

//***********************SMC Params*********************************************//
#define LOOPTIMEINSEC (1.0 / PWMFREQUENCY_HZ)                 // PWM Period = 1.0 / PWMFREQUENCY
#define SPEEDLOOPFREQ 1000                                    // 速度环频率（以赫兹为单位）。 此值必须 \
                                                              // 为整数以避免预编译器错误
#define SPEEDLOOPTIME (float)(1.0 / SPEEDLOOPFREQ)            // 调速周期
#define IRP_PERCALC (uint16_t)(SPEEDLOOPTIME / LOOPTIMEINSEC) // 每个速度计算的PWM回路
#define TRANSITION_STEPS IRP_PERCALC / 4

#define SMCGAIN 0.85       // Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC 0.005 // If measured current - estimated current   \
                           // is less than MAXLINEARSMC, the slide mode \
                           // Controller will have a linear behavior    \
                           // instead of ON/OFF. Value from (0.0 to 0.9999)

#define STARTUPRAMP_THETA_OPENLOOP_SCALER 10

#define MAX_VOLTAGE_VECTOR 0.98
// Vd and Vq vector limitation variables

#define SMO_SPEED_EST_MULTIPLIER Q15(0.9155273)

#define THETA_FILTER_CNST Q15(0.104719 * LOOPTIME_SEC * 32768.0) //2*pi/60*Ts*32768

//***********************End of SMC Params************************************//
#endif


