#ifndef _SMC_H
#define _SMC_H
#include "common.h"
// Define this in Degrees, from 0 to 360
#define THETA_AT_ALL_SPEED 90
#define THETA_ALL (uint16_t)(THETA_AT_ALL_SPEED / 180.0 * 32768.0)
#define CONSTANT_PHASE_SHIFT THETA_ALL
typedef struct
{
    int16_t MaxVoltage;
    int16_t Valpha;      //  静态α轴定子电压
    int16_t Ealpha;      //  静态α轴反电动势
    int16_t EalphaFinal; //  滤波后的EMF用于角度计算
    int16_t Zalpha;      //  静态α轴滑膜校准因子
    int16_t Gsmopos;     //  电机相关控制增益
    int16_t EstIalpha;   //  估算的静态α轴定子电流
    int16_t Fsmopos;     //  电机相关控制增益
    int16_t Vbeta;       //  静态β轴定子电压
    int16_t Ebeta;       //  静态的β轴反电动势
    int16_t EbetaFinal;  //  滤波后的EMF用于角度计算
    int16_t Zbeta;       //  静态β轴滑膜校准因子
    int16_t EstIbeta;    //  估算的静态β轴定子电流
    int16_t Ialpha;      //  静态α轴定子电流
    int16_t IalphaError; //  静态α轴定子电流与估算电流误差
    int16_t Kslide;      //  滑膜控制器增益
    int16_t MaxSMCError; //  最大电流误差
    int16_t Ibeta;       //  静态β轴定子电流
    int16_t IbetaError;  //  静态β轴电流误差
    int16_t Kslf;        //  滑膜滤波器增益
    int16_t KslfFinal;   //  用于角度计算的BEMF滤波器
    int16_t FiltOmCoef;  //  过滤系数，用于Omega过滤的calc
    int16_t ThetaOffset; //  偏移量用于补偿转子角度
    int16_t Theta;       //  输出：补偿转子角
    int16_t mdbi;        //  Kslide / MaxSMCError
    int16_t Omega;       //  转子转速
    int16_t OmegaFltred; //  转速PI的滤波转子转速
} SMC;
typedef SMC *SMC_handle;

/* Motor Estimator Parameter data type

  Description:
    This structure will host motor parameters parameters required by angle
    estimator.
 */
typedef struct
{
    /* Rs value - stator resistance */
    int16_t qRs;
    /* Ls/dt value - stator inductance / dt - variable with speed */
    int16_t qLsDt;
    /* Ls/dt value - stator inductance / dt for base speed (nominal) */
    int16_t qLsDtBase;
    /* InvKfi constant value ( InvKfi = Omega/BEMF ) */
    int16_t qInvKFi;
    /* InvKfi constant - base speed (nominal) value */
    int16_t qInvKFiBase;
} MOTOR_ESTIM_PARM_T;

#define SMC_DEFAULTS                                                                 \
    {                                                                                \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }

extern uint16_t trans_counter;

extern int16_t PrevTheta; // Previous theta which is then subtracted from Theta to get
                          // delta theta. This delta will be accumulated in AccumTheta, and
                          // after a number of accumulations Omega is calculated.

extern int16_t AccumTheta; // Accumulates delta theta over a number of times

extern uint16_t AccumThetaCnt; // Counter used to calculate motor speed. Is incremented
extern MOTOR_ESTIM_PARM_T motorParm;
extern SMC smc;
void SMC_Init(SMC_handle);
void SMC_Position_Estimation(SMC_handle);
#endif
