#include "control.h"
#include "pwm.h"
#include "init.h"
#include "halless.h"
#include "adc.h"
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
    PortOutput_Config(0, 1, 0, 1, 0, 1); // 所有输出口为低
    // Fault_InitOverUnderVoltage(); // 过压保护
    mcState = mcInit;
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
    PortOutput_Config(0, 0, 0, 0, 0, 0); // 打开下管充电
    // PWMPortBrake(); //充电
    delay1ms(10);
    PortOutput_Config(0, 1, 0, 1, 0, 1);
    delay10us(5);
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
    PWMChangeDuty(HoldParm.PWMDutyCycle);
    PWMSwitchPhase();
    delay1ms(200);
    PortOutput_Config(0, 1, 0, 1, 0, 1);
    delay10us(100);
    // IPD(); // 定位需要根据电机调整
    // SFRPAGE = 0x02; // 清除所有中断标识位
    Adc_EnableIrq(); //使能Adc中断
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
    Gpio_WriteOutputIO(GpioPortC, GpioPin13, FALSE);
}
/*****************************************************************************
 函 数 名  : StartupDrag
 功能描述  : 启动，边强拖，边检测
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void StartupDrag(void)
{
    static uint16_t ADC_CNT = 0;
    static uint8_t Zero_CNT = 0;
    CheckZeroCrossing();
    if (Halless.Zero_Flag)
    {
        ADC_CNT = 0;
        Halless.Zero_Flag = 0; //此处用作标识位
        Halless.Check_Count = 0;
        if (++Zero_CNT >= 24) // 需要调整切入闭环的时间
        {
            Zero_CNT = 0;
            EnterRunInit();
            return;
        }
    }
    else if (++ADC_CNT >= HoldParm.DragTime) // 需要调整强拖时间
    {
        ADC_CNT = 0;
        Zero_CNT = 0;
        Halless.Check_Count = 0;
        Halless.BackEMFFilter = 0;
        HoldParm.DragTime -= ((HoldParm.DragTime / 15) + 1); // 需要调整强拖加速
        if (HoldParm.DragTime < 300)
        {
            HoldParm.DragTime = 300;
        }
        if (++Halless.Phase > 5)
        {
            Halless.Phase = 0;
        }
        PWMSwitchPhase();
        HoldParm.PWMDutyCycle += 1;
        UP16LIMIT(HoldParm.PWMDutyCycle, PWM_DUTYCYCLE_25, PWM_START_DUTY);
        PWMChangeDuty(HoldParm.PWMDutyCycle);
    }
}
/*****************************************************************************
 函 数 名  : MotorRun
 功能描述  : PID调速
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorRun(void)
{
    if (HoldParm.SpeedLoopCnt > 1000)
    {
        HoldParm.SpeedLoopCnt = 0;
        // PI.FB = HoldParm.RPM;
        // // PID_CLAC(&PI);
        // HoldParm.PWMDutyCycle += PI.Out;
        // UP16LIMIT(HoldParm.PWMDutyCycle, PWM_DUTYCYCLE_50, PWM_START_DUTY);
        // PWMChangeDuty(HoldParm.PWMDutyCycle);
        HoldParm.PWMDutyCycle += 1;
        UP16LIMIT(HoldParm.PWMDutyCycle, PWM_DUTYCYCLE_50, PWM_MIN_DUTY);
        PWMChangeDuty(HoldParm.PWMDutyCycle);
        // printf("RPM=%d\r\n", HoldParm.RPM);
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
    PortOutput_Config(0, 1, 0, 1, 0, 1);
    Gpio_WriteOutputIO(GpioPortC, GpioPin13, TRUE);
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
