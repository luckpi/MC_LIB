#ifndef _COMMON_H
#define _COMMON_H
#include "ddl.h"
#define CW 1
#define CCW -1
// 控制顺序
typedef enum
{
    mcStop = 0, // 电机停止工作
    mcAhead,    // 启动检测
    mcInit,     // 启动初始化
    mcAlign,    // 定位
    mcDrag,     // 强拖启动
    mcRun,      // 进入闭环
    mcReset,    // 重置
    mcFault,    // 电机重启
} MotorState_T;

// 打印错误信息
typedef enum
{
    normal = 0,   // 运行正常
    overcurrent,  // adc过流保护
    cmp_isr,      // cmp过流保护
    overvoltage,  // 过压
    undervoltage, // 欠压
    stall,        // 堵转
} ErrorState_T;

// ADC相关
typedef struct
{
    int16_t POT;           // 电位器
    int16_t Voltage;       // 电压采样值
    int16_t Current;       // 电流采样值
    uint8_t OverVoltageCnt; // 过压次数
    uint8_t OverCurrentCnt; // 过流次数
} ADCSamplePara_T;

// PWM捕获相关
typedef struct
{
    uint16_t Period;      // 周期
    uint16_t Duty;        // 电平时间
    uint16_t PWMCnt;      // PWM信号滤波
    uint8_t DutyCycleUse; // 占空比
    uint8_t Flag_Cap_Valid : 1;
} PWMCatchPara_T;

extern MotorState_T mcState;
extern ErrorState_T error_code;
extern volatile PWMCatchPara_T CatchParm;
extern volatile ADCSamplePara_T ADCSample;
extern void Common_Init(void);
extern void PowerupParaInit(void);
#endif
