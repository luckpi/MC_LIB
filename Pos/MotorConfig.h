#ifndef _MOTORCFG_H
#define _MOTORCFG_H
#include "common.h"
/******************** PWM and Control Timing Parameters *********************/
/* PWM频率 */
#define PWM_FREQ                    16000
/* 死区时间 */
#define PWM_DTS                     0.0000002

/* PWM周期 (1 / PWM_FREQ) */
#define PWM_TS                      0.0000625

/****************************** Motor Parameters ******************************/
#define MAX_MOTOR_VOLTAGE           24      // 最大电压 V
#define MAX_MOTOR_CURRENT           15      // 最大电流 A
#define MOTOR_RS                    1       // 定子电阻 Ω
#define MOTOR_LS                    0.00268 // 定子电感 H
#define NOPOLESPAIRS                4       // 极对数


#define END_SPEED_RPM               500     // 开环结束切闭环速度
#define NOMINAL_SPEED_RPM           2700    // 电机额定转速(RPM)
#define MAXIMUM_SPEED_RPM           5300    // 电动机的最大转速(RPM)
#define FW_NOMINAL_SPEED_RPM        2700

// FG增益
#define NORM_LSDTBASE_SCALINGFACTOR 8
#define NORM_RS_SCALINGFACTOR       1

/* 开环结束切闭环速度转换为电气速度 */
#define ENDSPEED_ELECTR             END_SPEED_RPM *NOPOLESPAIRS

#endif
