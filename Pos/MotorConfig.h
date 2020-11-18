#ifndef _MOTORCFG_H
#define _MOTORCFG_H
#include "common.h"
#define MAX_MOTOR_CURRENT 15 //最大电流
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

#endif
