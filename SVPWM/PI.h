#ifndef _PI_H
#define _PI_H
#include "common.h"
#include "MotorConfig.h"
#include "atan2.h"
#define PWM_FREQ 16000
#define PWM_TS (float)(1.0 / PWM_FREQ)
#define RL_WC_VELREF_FIL ((float)1.0)                                                                /* cross frequency of velocity reference filter */
#define RL_WCTS_VELREF (float)(RL_WC_VELREF_FIL * PWM_TS)                                            /* Wc*Ts */
#define RL_1MINUS_WCTS_VELREF (float)(1.0 - RL_WCTS_VELREF)                                          /* 1 - Wc*Ts */
#define MAX_ADC_COUNT (float)4095                                                                    /* 12-bit ADC */
#define NOMINAL_SPEED_RAD_PER_SEC_ELEC (float)(((NOMINAL_SPEED_RPM / 60) * 2 * M_PI) * NOPOLESPAIRS) // Value in RPM
#define POT_ADC_COUNT_FW_SPEED_RATIO (float)(NOMINAL_SPEED_RAD_PER_SEC_ELEC / MAX_ADC_COUNT)
#define Q_CURRENT_REF_OPENLOOP 13107                                                      //((float)0.4) 启动-电动机在电流控制模式下启动加速
#define SINGLE_ELEC_ROT_RADS_PER_SEC (float)(2 * M_PI)                                    // 360°
#define END_SPEED_RPS ((float)END_SPEED_RPM / 60)                                         // 结束时的转速   单位：s
#define END_SPEED_RADS_PER_SEC_MECH (float)(END_SPEED_RPS * SINGLE_ELEC_ROT_RADS_PER_SEC) // 单对级电机转速转角度
#define END_SPEED_RADS_PER_SEC_ELEC (float)(END_SPEED_RADS_PER_SEC_MECH * NOPOLESPAIRS)   // 电机转速转角度
/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define D_CURRCNTR_PTERM 655    //0.02
#define D_CURRCNTR_ITERM 2      //(0.00005)
#define D_CURRCNTR_CTERM 16384  //0.5
#define D_CURRCNTR_OUTMAX 32734 //0.99

//******** Q Control Loop Coefficients *******
#define Q_CURRCNTR_PTERM 655    //0.02
#define Q_CURRCNTR_ITERM 2      //(0.00005)
#define Q_CURRCNTR_CTERM 16384  //0.5
#define Q_CURRCNTR_OUTMAX 32734 //0.999

//*** Velocity Control Loop Coefficients *****
#define SPEEDCNTR_PTERM 164   //(0.005)
#define SPEEDCNTR_ITERM 1     //(0.0000006)
#define SPEEDCNTR_CTERM 16384 //0.5
#define SPEEDCNTR_OUTMAX MAX_MOTOR_CURRENT

typedef struct
{
    int16_t qdSum;
    int16_t qKp;
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


