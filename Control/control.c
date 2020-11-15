#include "control.h"
#include "pwm.h"
#include "init.h"
#include "halless.h"
#include "adc.h"
#include "IQmath.h"
#include "svgen_dq.h"
// #include "protect.h"
// #include "cmp.h"
// #include "timer.h"
// #include "uart.h"
// #include "debug.h"
// #include "ipd.h"
// #include "pid.h"
/******************************************************************************
 函 数 名  : MotorAhead
 功能描述  : 开始启动
 输入参数  : 无
 输出参数  : void
******************************************************************************/
static void MotorAhead(void)
{
    Adc_EnableIrq(); // 使能Adc中断 放在强拖之前开
    // Fault_InitOverUnderVoltage(); // 过压保护
}
/******************************************************************************
 函 数 名  : MotorInit
 功能描述  : 初始化
 输入参数  : 无
 输出参数  : void
******************************************************************************/
static void MotorInit(void)
{
    Common_Init(); // 变量初始化
    // ALL_INT_DISEN; // 关闭所有中断
    // PortOutput_Config(0, 1, 0, 1, 0, 1); // 打开下管充电
    // PWMPortBrake(); //充电
    delay1ms(10);
    // PortOutput_Config(0, 0, 0, 0, 0, 0);
    delay10us(10);
    mcState = mcAlign;
}

/*****************************************************************************
 函 数 名  : MotorAlign
 功能描述  : 定位初始相位，开启ADC中断，准备进入强拖启动
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorAlign(void)
{
    HoldParm.PWMDutyCycle = PWM_START_DUTY;
    //    PWMChangeDuty(HoldParm.PWMDutyCycle);

    delay1ms(100);
    delay10us(200);
    // IPD(); // 定位需要根据电机调整
    // SFRPAGE = 0x02; // 清除所有中断标识位
    PortOutput_Config(6, 6, 6, 6, 6, 6);
    mcState = mcDrag;
}
/*****************************************************************************
 函 数 名  : EnterRunInit
 功能描述  : 进入Run的初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void EnterRunInit(void)
{
    mcState = mcRun;
}
/*****************************************************************************
 函 数 名  : StartupDrag
 功能描述  : 启动，边强拖，边检测
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void StartupDrag(void)
{
    // static uint16_t ADC_CNT = 0;
    // if (++ADC_CNT >= 50)
    // {
    //     ADC_CNT = 0;
    SVM.Vd = 10000;
    SVM.Vq = 0;
    AngleSin_Cos.IQAngle += 15;
    if (AngleSin_Cos.IQAngle > 32767)
    {
        AngleSin_Cos.IQAngle = -32768;
    }
    InvPark();
    svgendq_calc();
    PWMChangeDuty((uint16_t)SVM.Ta, (uint16_t)SVM.Tb, (uint16_t)SVM.Tc);
    // }
}
/*****************************************************************************
 函 数 名  : MotorRun
 功能描述  : PID调速
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorRun(void)
{
    if (HoldParm.SpeedLoopCnt > 500)
    {
        HoldParm.SpeedLoopCnt = 0;
    }
}

/*****************************************************************************
 函 数 名  : MotorStop
 功能描述  : 电机停止转动
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void MotorStop(void)
{
    // PortOutput_Config(0, 0, 0, 0, 0, 0);
}

/*****************************************************************************
 函 数 名  : MotorFault
 功能描述  : 电机等待重启状态，电流保护，CMP保护，堵转保护。
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorFault(void)
{
    //    Debug(); // 打印错误信息
    //    Ps("\nloading reboot\n", NONE);
}

/*****************************************************************************
 函 数 名  : MotorControl
 功能描述  : 电机控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void MotorControl(void)
{
    switch (mcState)
    {
    case mcAhead:
        MotorAhead();
        break;
    case mcInit:
        MotorInit();
        break;
    case mcAlign:
        MotorAlign();
        break;
    case mcDrag:
        break;
    case mcRun:
        MotorRun();
        break;
    case mcStop:
        MotorStop();
        break;
    case mcFault:
        MotorFault();
        break;
    default:
        MotorStop();
        break;
    }
}
