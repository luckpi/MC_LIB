#ifndef _PI_H
#define _PI_H
#include "common.h"
#define Q_CURRENT_REF_OPENLOOP               2000    // 启动力矩  I / 最大电流
#define NKo                                  16     // KP增益
/* PI 控制器调整值 */
/********** D轴控制环路系数 **********/
#define D_CURRCNTR_PTERM                     0.05
#define D_CURRCNTR_ITERM                     0.005
#define D_CURRCNTR_CTERM                     0.999
#define D_CURRCNTR_OUTMAX                    0.8

/********** Q轴控制环路参数 **********/
#define Q_CURRCNTR_PTERM                     0.05
#define Q_CURRCNTR_ITERM                     0.005
#define Q_CURRCNTR_CTERM                     0.999
#define Q_CURRCNTR_OUTMAX                    0.8    

/********** 闭环速度环路参数 **********/
#define SPEEDCNTR_PTERM                      0.2
#define SPEEDCNTR_ITERM                      0.0025
#define SPEEDCNTR_CTERM                      0.9999
#define SPEEDCNTR_OUTMAX                     0.8
#define SPEEDREFRAMP                         1
#define SPEEDREFRAMP_COUNT                   3

typedef struct
{
    int32_t qdSum;
    int32_t qKp;
    int16_t qKi;
    int16_t qKc;
    int16_t qOutMax;
    int16_t qOutMin;
    int16_t qInRef;
    int16_t qInMeas;
    int16_t qOut;
    int16_t qErr;
} tPIParm;

typedef struct
{
    // 开环控制
    int32_t OpenLoopSpeed;    // 开环速度
    int32_t OpenLoopSpeedEnd; // 开环结束速度
    int16_t OpenLoopSpeedAdd; // 开环速度增量
    int32_t OmegaMin;         // 最小转速
    int32_t OmegaMax;         // 最大转速
    int16_t Theta_error;      // 开环角和估算角的误差
    uint16_t trans_counter;   // 减小开环角和估算角差距间隔
    uint16_t SpeedLoop_FLAG;  // 速度环标识位

    // 闭环控制
    int16_t VelRef;         // 参考速度
    int16_t IdRef;          // Vd磁通参考值
    int16_t IqRef;          // Vq转矩参考值
    int16_t qRefRamp;       // 斜坡速度参考值
    int16_t qDiff;          // 坡道速度
    int16_t IdRefFF;        // 弱磁期间前馈Idref
    int16_t IdRefPI;        // 弱场Idref PI输出
    int16_t IqRefmax;       // 最大Q轴电流
    int16_t SpeedRampCount; // 速度环PI周期
    int16_t RotorDirection; // 电机转向
} tCtrlParm;

extern tCtrlParm CtrlParm;
extern tPIParm PIParmQ;    /* Q轴电流PI控制器的参数 */
extern tPIParm PIParmD;    /* D轴电流PI控制器的参数 */
extern tPIParm PIParmQref; /* 速度PI控制器的参数 */
extern void PI_Init(void);
extern void PI_Control(void);

#endif
