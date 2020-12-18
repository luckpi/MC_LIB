#include "PI.h"
#include "smc.h"
#include "pwm.h"
#include "init.h"
#include "IQmath.h"
#include "halless.h"
#include "control.h"
#include "svgen_dq.h"
#include "MotorConfig.h"
/******************************************************************************
 函 数 名  : MotorAhead
 功能描述  : 开始启动
 输入参数  : 无
 输出参数  : void
******************************************************************************/
static void MotorAhead(void)
{
    Adc_EnableIrq(); // 使能Adc中断 放在强拖之前开
}
/******************************************************************************
 函 数 名  : MotorInit
 功能描述  : 初始化
 输入参数  : 无
 输出参数  : void
******************************************************************************/
static void MotorInit(void)
{
    PortOutput_Config(7, 7, 7, 7, 7, 7);
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
    uint16_t LockTime = 4000;
    if (HoldParm.MainDetectCnt > LockTime)
    {
        mcState = mcDrag;
    }
    return;
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
 函 数 名  : CalculateParkAngle
 功能描述  : 启动，边强拖，边检测
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CalculateParkAngle(void)
{
    if (mcState == mcDrag)
    {
        if (CtrlParm.OpenLoopSpeed < CtrlParm.OpenLoopSpeedEnd)
        {
            CtrlParm.OpenLoopSpeed += CtrlParm.OpenLoopSpeedAdd;
        }
        else
        {
            Theta_error = AngleSin_Cos.IQAngle - smc.Theta;
            PIParmQref.qdSum = CtrlParm.IqRef >> 3;
            CtrlParm.VelRef = CtrlParm.OmegaMin;
            PIParmD.qInRef = 0;
            CtrlParm.IdRef = 0;
            mcState = mcRun;
        }
        AngleSin_Cos.IQAngle += (int16_t)(CtrlParm.OpenLoopSpeed >> THETA_OPENLOOP_SCALER) * HoldParm.RotorDirection;
    }
    else if (mcState == mcRun)
    {
        if ((Abs(Theta_error) > _0_05DEG) && (trans_counter == 0)) // 慢慢减小开环强制角度和估算角度误差
        {
            if (Theta_error < 0)
                Theta_error += _0_05DEG;
            else
                Theta_error -= _0_05DEG;
        }
        AngleSin_Cos.IQAngle = smc.Theta + Theta_error;
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
    PortOutput_Config(0, 0, 0, 0, 0, 0);
    Common_Init(); // 变量初始化
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
