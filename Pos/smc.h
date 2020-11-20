#ifndef _SMC_H
#define _SMC_H
#include "common.h"

#define THETA_AT_ALL_SPEED              90              // 延迟角度  (0 ~ 360)
#define THETA_ALL                       16384           // (uint16_t)(THETA_AT_ALL_SPEED / 180.0 * 32768.0)
#define CONSTANT_PHASE_SHIFT THETA_ALL

/* Sliding Mode Control Parameter data type */
#define LOOPTIMEINSEC                   PWM_TS    // PWM Period = 1.0 / PWMFREQUENCY
#define SPEEDLOOPFREQ                   1000            // 速度环频率（以赫兹为单位）。 此值必须为整数以避免预编译器错误

#define SPEEDLOOPTIME                   0.001           // 调速周期 (1.0 / SPEEDLOOPFREQ)
#define IRP_PERCALC                     16              // 每个速度计算的PWM回路 (SPEEDLOOPTIME / LOOPTIMEINSEC)
#define TRANSITION_STEPS                4               // IRP_PERCALC / 4

#define SMCGAIN                         0.85            // 滑模控制器增益 (0.0 to 0.9999)
#define MAXLINEARSMC                    0.005           // 滑膜最大误差值域 (0.0 to 0.9999)

#define SMO_SPEED_EST_MULTIPLIER        30000           // Q15(0.9155273)

#define THETA_FILTER_CNST               7028            // Q15(0.104719 * PWM_TS * 32768.0) // 2*pi/60*Ts*32768


typedef struct
{
    int16_t MaxVoltage;   //  最大矢量电压
    int16_t Valpha;       //  α轴定子电压
    int16_t Ealpha;       //  α轴反电动势
    int16_t EalphaFinal;  //  滤波后的EMF用于角度计算
    int16_t Zalpha;       //  α轴滑膜校准因子
    int16_t Gsmopos;      //  电机相关控制增益
    int16_t EstIalpha;    //  估算的α轴定子电流
    int16_t Fsmopos;      //  电机相关控制增益
    int16_t Vbeta;        //  β轴定子电压
    int16_t Ebeta;        //  β轴反电动势
    int16_t EbetaFinal;   //  滤波后的EMF用于角度计算
    int16_t Zbeta;        //  β轴滑膜校准因子
    int16_t EstIbeta;     //  估算的β轴定子电流
    int16_t Ialpha;       //  α轴定子电流
    int16_t IalphaError;  //  α轴定子电流与估算电流误差
    int16_t Kslide;       //  滑膜控制器增益
    int16_t MaxSMCError;  //  最大电流误差
    int16_t Ibeta;        //  β轴定子电流
    int16_t IbetaError;   //  β轴电流误差
    int16_t Kslf;         //  滑膜滤波器增益
    int16_t KslfFinal;    //  用于角度计算的BEMF滤波器
    int16_t FiltOmCoef;   //  过滤系数，用于Omega过滤的calc
    int16_t ThetaOffset;  //  偏移量用于补偿转子角度
    uint16_t Theta;       //  输出：补偿转子角
    int16_t mdbi;         //  Kslide / MaxSMCError
    int16_t Omega;        //  转子转速
    int16_t OmegaFltred;  //  转速PI的滤波转子转速
    uint8_t OpenLood : 1; //  开环标识位
} SMC;
typedef SMC *SMC_handle;


/* Motor Estimator Parameter data type */
typedef struct
{
    /* Rs值-定子电阻 */
    int16_t qRs;
    /* Ls/dt value - 定子电感/ dt-随速度变化 */
    int16_t qLsDt;
    /* Ls/dt value - 定子电感/ dt（基本速度）（标称） */
    int16_t qLsDtBase;

    float Vol_Const;

    float Cur_Const;

    float Omg_Const;
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
extern uint16_t PrevTheta; 
extern int16_t AccumTheta; 
extern uint16_t AccumThetaCnt; 
extern MOTOR_ESTIM_PARM_T motorParm;
extern SMC smc;

extern void SMC_Init(SMC_handle);
extern void SMC_Position_Estimation(SMC_handle);

#endif
