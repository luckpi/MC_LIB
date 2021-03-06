#ifndef _MOTORCFG_H
#define _MOTORCFG_H
#include "common.h"
/******************** PWM and Control Timing Parameters *********************/
//  PWM配置
#define PWMSYSCLK                   48000000  // PWM时钟
#define SYSCLK                      48000000  // 除PWM外的模块时钟
#define PWM_FRE_SETATA              1200      //自动重装载值((PWMSYSCLK / PWM_FRE) >> 1)

/* PWM频率 */
#define PWM_FREQ                    20000
/* 死区时间 */
#define PWM_DTS                     0.000002
/* PWM周期 (1 / PWM_FREQ) */
#define PWM_TS                      0.000050

/****************************** Motor Parameters ******************************/
#define MAX_MOTOR_VOLTAGE           24          // 最大电压 V
#define MAX_MOTOR_CURRENT           15.625      // 最大电流 A  = ADC参考电压 / 2 / (运放倍数 x 采样电阻大小)
#define MOTOR_RS                    0.75        // 定子电阻 Ω
#define MOTOR_LS                    0.00175     // 定子电感 H
#define NOPOLESPAIRS                4           // 极对数
#define MAX_VOLTAGE_VECTOR          30000       // 电压矢量最大值 Q15

#define END_SPEED_RPM               500         // 开环结束切闭环速度
#define NOMINAL_SPEED_RPM           3000        // 电机额定转速(RPM)
#define MAXIMUM_SPEED_RPM           5300        // 电动机的最大转速(RPM)
#define FW_NOMINAL_SPEED_RPM        3000        // 弱磁额定转速
#define ENDSPEED_ELECTR             (END_SPEED_RPM * NOPOLESPAIRS)

// 转矩模式 开启会跳过速度环。注释关闭
// #define TORQUE_MODE

// 弱磁功能 注释关闭
// #define FDWEAK_MODE

/****************************** Open Lood Parameters ******************************/
#define THETA_OPENLOOP_SCALER        16         // 缩放加速度
#define OPENLOOP_TIME                6          // 开环时间 S
/* 开环结束切闭环速度转换为电气速度 */
#define END_SPEED                    (ENDSPEED_ELECTR * PWM_TS * 65536 / 60.0) // 每个周期变化的角度 
#endif
