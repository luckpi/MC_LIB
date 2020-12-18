#ifndef _PI_H
#define _PI_H
#include "common.h"
#define Q_MAX                                27000  // Q轴闭环最大电流
#define Q_CURRENT_REF_OPENLOOP               600   // 启动力矩  I / 最大电流
/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define D_CURRCNTR_PTERM                     0.05
#define D_CURRCNTR_ITERM                     0.005
#define D_CURRCNTR_CTERM                     0.999
#define D_CURRCNTR_OUTMAX                    0.8

//******** Q Control Loop Coefficients *******
#define Q_CURRCNTR_PTERM                     0.05
#define Q_CURRCNTR_ITERM                     0.005
#define Q_CURRCNTR_CTERM                     0.999
#define Q_CURRCNTR_OUTMAX                    0.8    

//*** Velocity Control Loop Coefficients *****
#define SPEEDCNTR_PTERM                      0.05
#define SPEEDCNTR_ITERM                      0.005
#define SPEEDCNTR_CTERM                      0.9999
#define SPEEDCNTR_OUTMAX                     0.5
#define SPEEDREFRAMP                         2
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
    int16_t VelRef;   // 参考速度
    int16_t IdRef;    // Vd磁通参考值
    int16_t IqRef;    // Vq转矩参考值
    int16_t qRefRamp; // 斜坡速度参考值
    int16_t qDiff;    // 坡道速度
    int16_t IdRefFF;  // 弱磁期间前馈Idref
    int16_t IdRefPI;  // 弱场Idref PI输出
    int16_t IqRefmax; // 最大Q轴电流
} tCtrlParm;
extern tCtrlParm CtrlParm;
extern tPIParm PIParmQ;    /* Q轴电流PI控制器的参数 */
extern tPIParm PIParmD;    /* D轴电流PI控制器的参数 */
extern tPIParm PIParmQref; /* 速度PI控制器的参数 */
extern void PI_Parameters(void);
extern void PI_Control(void);
#endif
