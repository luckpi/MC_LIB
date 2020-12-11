#ifndef _SMC_H
#define _SMC_H
#include "atan2.h"
#include "common.h"
#include "IQmath.h"
#include "MotorConfig.h"

#define ONE_BY_SQRT3                            0.5773502691    // 1 / √3
#define TWO_PI                                  6.283185307     // 2pi

#define THETA_AT_ALL_SPEED                      90              // 延迟角度  (0 ~ 360) 正转270 反转90
#define THETA_ALL                               16384         // (uint16_t)(THETA_AT_ALL_SPEED / 180.0 * 32768.0)
#define CONSTANT_PHASE_SHIFT                    THETA_ALL
#define _0_05DEG                                9               // 闭环减小开环强制角和估算角误差，每次步进0.05°

/* Sliding Mode Control Parameter data type */
#define SPEEDLOOPFREQ                           1000    // 速度环频率（以赫兹为单位）。 此值必须为整数以避免预编译器错误
#define SPEEDLOOPTIME                           0.001   // 调速周期 (1.0 / SPEEDLOOPFREQ)
#define IRP_PERCALC                             16      // 每个速度计算的PWM回路 (SPEEDLOOPTIME / PWM_TS)
#define TRANSITION_STEPS                        4       // IRP_PERCALC / 4
#define SMCGAIN                                 0.85     // 滑模控制器增益 (0.0 to 0.9999)
#define MAXLINEARSMC                            0.005   // 滑膜最大误差值域 (0.0 to 0.9999)
#define SMO_SPEED_EST_MULTIPLIER                30000   // Q15(0.9155273)
#define THETA_FILTER_CNST                       7028    // Q15(0.104719 * PWM_TS * 32768.0) // 2 * pi / 60 * Ts * 32768
typedef struct
{
    int16_t Valpha;      //  α轴定子电压
    int16_t Ialpha;      //  α轴定子电流
    int16_t EstIalpha;   //  估算的α轴定子电流
    int16_t Ealpha;      //  估算的α轴反电动势
    int16_t EalphaFinal; //  滤波后的反电动势用于反正切
    int16_t Zalpha;      //  α轴滑膜校准因子

    int16_t Vbeta;      //  β轴定子电压
    int16_t Ibeta;      //  β轴定子电流
    int16_t EstIbeta;   //  估算的β轴定子电流
    int16_t Ebeta;      //  估算的β轴反电动势
    int16_t EbetaFinal; //  滤波后的反电动势用于反正切
    int16_t Zbeta;      //  β轴滑膜校准因子

    int16_t Fsmopos;     //  电机相关控制增益
    int16_t Gsmopos;     //  电机相关控制增益
    int16_t Kslide;      //  滑膜控制器增益
    int16_t MaxSMCError; //  最大电流误差
    int16_t mdbi;        //  Kslide / MaxSMCError
    int16_t Kslf;        //  滑膜滤波器增益
    int16_t Kslf_min;    //  滑膜滤波器最小增益
    int16_t FiltOmCoef;  //  过滤系数，用于Omega过滤的calc
    int16_t ThetaOffset; //  偏移量用于补偿转子角度
    int16_t Theta;       //  转子补偿角
    int16_t Omega;       //  转子角速度
    int16_t OmegaFltred; //  用于速度环的转子角速度
} SMC, *p_SMC;

/* 电机归一化参数 */
typedef struct
{
    int32_t qRs;
    int32_t qLsDt;     // Ls / dt
    int32_t qLsDtBase; // Ls / dt
    float Vol_Const;
    float Cur_Const;
    float Omg_Const;
    // /* InvKfi constant value ( InvKfi = Omega/BEMF ) */
    // int16_t qInvKFi;
    // /* InvKfi constant - base speed (nominal) value */
    // int16_t qInvKFiBase;
} MOTOR_ESTIM, *p_MOTOR_ESTIM;

#define SMC_DEFAULTS                                                           \
    {                                                                          \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }

extern SMC smc;

extern int16_t PrevTheta;
extern int16_t AccumTheta;
extern int16_t Theta_error;
extern uint16_t AccumThetaCnt;
extern uint16_t trans_counter;
extern MOTOR_ESTIM motorParm;

extern void SMC_Init(p_SMC, p_MOTOR_ESTIM);
extern void SMC_Position_Estimation(p_SMC);
extern int16_t CORDIC_Atan(int16_t alfa_est, int16_t beta_est);

#endif
