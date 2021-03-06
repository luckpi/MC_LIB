#include "PI.h"
#include "smc.h"
#include "pwm.h"
#include "init.h"
#include "IQmath.h"
#include "control.h"
#include "svgen_dq.h"
#include "sensorless.h"
#include "MotorConfig.h"

#define _0_05DEG    9   // 缩减开环角与估算角的单位周期增量
#define STEPS       5   // 缩减开环角与估算角的周期

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
 功能描述  : 定位初始角度
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorAlign(void)
{
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
 功能描述  : 计算Park角度
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
            if (CtrlParm.OpenLoopSpeed >= CtrlParm.OpenLoopSpeedEnd)
            {
                // 计算开环角和估算角误差，开启速度环
                CtrlParm.Theta_error = svm.Theta - smc.Theta;
                PIParmQref.qdSum = CtrlParm.IqRef;
                CtrlParm.VelRef = CtrlParm.OmegaMin;
                CtrlParm.SpeedLoop_FLAG = 1;
                PIParmD.qInRef = 0;
                CtrlParm.IdRef = 0;
            }
            svm.Theta += (int16_t)(CtrlParm.OpenLoopSpeed >> THETA_OPENLOOP_SCALER) * CtrlParm.RotorDirection;
        }
        else
        {
            // 慢慢减小开环强制角度和估算角度误差
            if ((Abs(CtrlParm.Theta_error) > _0_05DEG))
            {
                if (++CtrlParm.trans_counter >= STEPS)
                {
                    if (CtrlParm.Theta_error < 0)
                        CtrlParm.Theta_error += _0_05DEG;
                    else
                        CtrlParm.Theta_error -= _0_05DEG;
                    CtrlParm.trans_counter = 0;
                }
            }
            else
            {
                mcState = mcRun;
            }
            svm.Theta = smc.Theta + CtrlParm.Theta_error;
        }
    }
    else if (mcState == mcRun)
    {
        svm.Theta = smc.Theta;
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
