#ifndef _SMC_H
#define _SMC_H
#include "common.h"
#include "IQmath.h"
#include "MotorConfig.h"
#include "svgen_dq.h"
#define ONE_BY_SQRT3                            0.5773502691    // 1 / √3

// 相位偏移补偿角
#define THETA_OFFSET                            90              // 45° * 2
#define CONSTANT_PHASE_SHIFT                    16384           // (THETA_OFFSET / 360) * 65535

// 滑模控制器参数
#define SPEEDLOOPFREQ                           1000            // 速度环频率 (Hz)
#define SPEEDLOOPTIME                           0.001           // 调速周期 (1.0 / SPEEDLOOPFREQ)
#define IRP_PERCALC                             20              // 每个速度计算的PWM回路 (SPEEDLOOPTIME / PWM_TS)
#define SMCGAIN                                 0.85            // 滑模控制器增益 (0.0 to 0.9999)
#define MAXLINEARSMC                            0.03            // 滑膜最大误差值域 (0.0 to 0.9999)
#define THETA_FILTER_CNST                       5147            // Q15(PI / IRP_PERCALC)

typedef struct
{
    int16_t EstIalpha;   // 估算的α轴定子电流
    int16_t IalphaError; // 估算的α轴电流误差
    int16_t Ealpha;      // 估算的α轴反电动势
    int16_t EalphaFinal; // 滤波后的反电动势用于反正切
    int16_t Zalpha;      // α轴滑膜校准因子

    int16_t EstIbeta;   // 估算的β轴定子电流
    int16_t IbetaError; // 估算的β轴电流误差
    int16_t Ebeta;      // 估算的β轴反电动势
    int16_t EbetaFinal; // 滤波后的反电动势用于反正切
    int16_t Zbeta;      // β轴滑膜校准因子

    int16_t Fsmopos;     // 电机数学模型F增益
    int16_t Gsmopos;     // 电机数学模型G增益
    int16_t Kslide;      // 滑膜控制器增益
    int16_t MaxSMCError; // 最大电流误差
    int32_t mdbi;        // 线性区增益系数
    int16_t Kslf;        // BEMF滤波系数
    int16_t Kslf_min;    // BEMF滤波系数最小值
    int16_t FiltOmCoef;  // Omega滤波系数
    int16_t ThetaOffset; // 相位偏移补偿角
    int16_t Theta;       // 估算角度
    int16_t Omega;       // 估算角速度
    int16_t OmegaFltred; // 滤波后的角速度
} SMC, *p_SMC;

// 电机归一化参数
typedef struct
{
    int32_t qRs;
    int32_t qLsDt; // Ls / dt
    int32_t Cur_Vol;
} MOTOR_ESTIM, *p_MOTOR_ESTIM;

#define SMC_DEFAULTS                                               \
    {                                                              \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }

extern SMC smc;
extern MOTOR_ESTIM motorParm;

extern int16_t PrevTheta;
extern int16_t AccumTheta;
extern uint16_t AccumThetaCnt;


extern void SMC_Init(p_SMC, p_MOTOR_ESTIM);
extern void SMC_Position_Estimation(p_SMC, p_SVGENDQ);
extern int16_t CORDIC_Atan(int16_t x, int16_t y);

#endif
