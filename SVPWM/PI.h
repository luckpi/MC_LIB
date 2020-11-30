#ifndef _PI_H
#define _PI_H
#include "common.h"
#define RL_WC_VELREF_FIL                     ((float)1.0)  /* 速度参考滤波器的交叉频率 */
#define Q_MAX                                27000
#define RL_WCTS_VELREF                       2      // (float)(RL_WC_VELREF_FIL * PWM_TS) /* Wc*Ts */
#define RL_1MINUS_WCTS_VELREF                32766  // (float)(1.0 - RL_WCTS_VELREF) /* 1 - Wc*Ts */
#define MAX_ADC_COUNT                        4095   // (float)4095   /* 12-bit ADC */
#define POT_ADC_COUNT_FW_SPEED_RATIO         1000   // 建议用角速度计算 / 4095
#define Q_CURRENT_REF_OPENLOOP               1200   // 启动力矩  I / 最大电流
/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define D_CURRCNTR_PTERM                     34000    //0.02
#define D_CURRCNTR_ITERM                     5      //(0.00005)
#define D_CURRCNTR_CTERM                     5      //0.5
#define D_CURRCNTR_OUTMAX                    27000  //0.999

//******** Q Control Loop Coefficients *******
#define Q_CURRCNTR_PTERM                     34000     //0.02KP=KP*2^NKo
#define Q_CURRCNTR_ITERM                     5      //(0.00005)
#define Q_CURRCNTR_CTERM                     5     //0.5
#define Q_CURRCNTR_OUTMAX                    27000  // 0.999

//*** Velocity Control Loop Coefficients *****
#define SPEEDCNTR_PTERM                      55
#define SPEEDCNTR_ITERM                      0
#define SPEEDCNTR_CTERM                      0
#define SPEEDCNTR_OUTMAX                     27000

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
