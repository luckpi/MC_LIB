#ifndef _COMMON_H
#define _COMMON_H
#include "ddl.h"
#define CW 1
#define CCW -1
#define First_Gear 1000  // 一档
#define Second_Gear 1500 // 二档
#define Third_Gear 2500  // 三档
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
    cmp_isr,      //cmp过流保护
    overvoltage,  // 过压
    undervoltage, // 欠压
    hallerror,    // 霍尔错误
    stall,        // 堵转
} ErrorState_T;

// 电机运行方向、占空比等参数
typedef struct
{
    int8_t RotorDirection;  // 电机转动方向
    uint16_t MainDetectCnt; // 主循环计数
    uint16_t SpeedLoopCnt;  // 调整转速占空比的周期计数
    float PWMDutyCycle;     // PWM占空比
    uint16_t SpeedTime;     // 一个电周期时间
    uint16_t RPM;           // 实际转速
    uint16_t Set_RPM;       // 设定转速
    uint16_t DragTime;      // 强拖时间
    uint32_t SpeedTime_Sum; // 换相时间8次和
    uint8_t SpeedTime_Cnt;  // 换相平均数计数
    uint16_t SpeedTimeTemp; // 当前换相时间
    uint8_t Stall_Cnt;      // 堵住计数溢出次数
} HoldControlPara_T;

// ADC相关
typedef struct
{
    uint8_t ChlState;       // 通道状态
    uint16_t CurrentOffset; // 电流放大零点
    uint32_t POT;           // 电位器
    uint32_t Voltage;       // 电压采样值
    uint16_t Current;       // 电流采样值
    uint32_t Sum;           // 电流平均值和
    uint8_t Num;            // 电流平均值计数
    uint16_t Average;       // 电流平均值
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
extern volatile HoldControlPara_T HoldParm;
extern volatile ADCSamplePara_T ADCSample;
extern void Common_Init(void);
extern void PowerupParaInit(void);
#endif
