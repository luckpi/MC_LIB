#include "smc.h"
#include "atan2.h"
#include "IQmath.h"
#include "MotorConfig.h"
#define ONE_BY_SQRT3 0.5773502691
#define TWO_PI 6.283185307
uint16_t trans_counter = 0;
int16_t Theta_error = 0;    // 开环强制角度和估算角度的误差,在闭环的过程中慢慢减去误差,每次步进0.05度。
uint16_t PrevTheta = 0;     // 上一次角度值
int16_t AccumTheta = 0;     // 累加每次的角度变化量
uint16_t AccumThetaCnt = 0; // 用于计算电机速度的计数器.
MOTOR_ESTIM_PARM_T motorParm;
SMC smc = SMC_DEFAULTS;
/*****************************************************************************
 函 数 名  : SMC_Init
 功能描述  : 滑膜控制器参数初始化
 输入参数  : 滑膜参数结构体地址
 输出参数  : void
*****************************************************************************/
void SMC_Init(SMC *s)
{
    // 电机参数归一化
    motorParm.Vol_Const = MAX_MOTOR_VOLTAGE * ONE_BY_SQRT3 / 32768.0 * (1.0 - PWM_DTS / PWM_TS);
    motorParm.Cur_Const = MAX_MOTOR_CURRENT / 32768.0;
    motorParm.Omg_Const = TWO_PI / 60.0;
    motorParm.qLsDtBase = (Q15(MOTOR_LS / motorParm.Vol_Const * motorParm.Cur_Const / PWM_TS)) >> 8;
    motorParm.qLsDt = motorParm.qLsDtBase;
    motorParm.qRs = (Q15(MOTOR_RS / motorParm.Vol_Const * motorParm.Cur_Const)) >> 1;
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //             Ts
    // Gsmopos = ------
    //              L
    // Ts = 采样周期。 如果以PWM采样, Ts = 62.5 us
    // R  = 相位电阻。 如果电机数据表未提供，用万用表测量相位电阻,除以二得到相位电阻
    // L  = 相位电感。 如果电机数据表未提供，用万用表测量相位电感,除以二得到相位电感
    if (((int32_t)motorParm.qRs << NORM_RS_SCALINGFACTOR) >= ((int32_t)motorParm.qLsDt << NORM_LSDTBASE_SCALINGFACTOR))

        s->Fsmopos = 0;
    else
        s->Fsmopos = (0x7FFF - HDIV_div(((int32_t)motorParm.qRs << (15 + NORM_RS_SCALINGFACTOR - NORM_LSDTBASE_SCALINGFACTOR)), motorParm.qLsDt));

    if (((int32_t)motorParm.qLsDt << NORM_LSDTBASE_SCALINGFACTOR) < 32767)
        s->Gsmopos = 0x7FFF;
    else
        s->Gsmopos = HDIV_div(((int32_t)0x7FFF << (15 - NORM_LSDTBASE_SCALINGFACTOR)), motorParm.qLsDt);

    s->Kslide = Q15(SMCGAIN);
    s->MaxSMCError = Q15(MAXLINEARSMC);
    s->mdbi = HDIV_div(s->Kslide, s->MaxSMCError);
    s->FiltOmCoef = (int16_t)(_IQmpy(ENDSPEED_ELECTR, THETA_FILTER_CNST));
    // s->MaxVoltage = (int16_t)(_IQmpy(ADCSample.Voltage, 18918));//_IQ(0.57735026918963)
    return;
}
/*****************************************************************************
 函 数 名  : CalcBEMF
 功能描述  : 估算反电动势滤波
 输入参数  : *EMF , *EMFF,  Z
 输出参数  : void
*****************************************************************************/
void CalcBEMF(int16_t *EMF, int16_t *EMFF, int16_t Z)
{
    int16_t temp_int1, temp_int2;
    temp_int1 = (int16_t)(_IQmpy(smc.Kslf, Z));
    temp_int2 = (int16_t)(_IQmpy(smc.Kslf, (*EMF)));
    temp_int1 -= temp_int2;
    (*EMF) += temp_int1;
    temp_int1 = (int16_t)(_IQmpy(smc.Kslf, (*EMF)));
    temp_int2 = (int16_t)(_IQmpy(smc.Kslf, (*EMFF)));
    temp_int1 -= temp_int2;
    (*EMFF) += temp_int1;
}
/*
电流观测器：
电机物理模型：Vs = R * Is + L * d(Is)/dt + Es  
电流电流模型：d(Is) / dt = (-R/L) * Is + 1/L * (Vs - Es) 
电流数学表达式：Is(n+1) = (1 - Ts * R / L) * Is(n) + Ts / L * (Vs(n) - Es(n))
F = (1 - Ts * R / L)，G = Ts / L (两个增益函数和电机特性有关)
(Us:输入电压矢量，Is:电机电流矢量，R:绕组电感，L:绕组电感，Ts:控制周期，Es:反电动势矢量)

输入参数：
I   ：经过Clark变换后的实际电流
U   ：Vbus / √3 * Valpha(Vbeta)  
EMF： 估算的反电动势
EstI：估算的电流
z   : 校准因子
*/
void CalcEstI(int16_t U, int16_t I, int16_t EMF, int16_t *EstI, int16_t *Z)
{
    int16_t temp_int1, temp_int2, temp_int3, I_Error;
    temp_int1 = (int16_t)(_IQmpy(smc.Gsmopos, U));
    temp_int2 = (int16_t)(_IQmpy(smc.Gsmopos, EMF));
    temp_int3 = (int16_t)(_IQmpy(smc.Gsmopos, (*Z)));
    temp_int1 -= temp_int2;
    temp_int1 -= temp_int3;
    *EstI = temp_int1 + (int16_t)(_IQmpy(smc.Fsmopos, (*EstI)));
    I_Error = (*EstI) - I;
    if (Abs(I_Error) < smc.MaxSMCError)
    {
        // s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
        (*Z) = (smc.mdbi * I_Error);
    }
    else if (I_Error > 0)
    {
        (*Z) = smc.Kslide;
    }
    else
    {
        (*Z) = -smc.Kslide;
    }
}
/*****************************************************************************
 函 数 名  : SMC_Position_Estimation
 功能描述  : 滑膜控制器，估算角度
 输入参数  : 滑膜参数结构体地址
 输出参数  : void
*****************************************************************************/
void SMC_Position_Estimation(SMC *s)
{
    int16_t Kslf_min;
    CalcEstI(smc.Valpha, smc.Ialpha, smc.Ealpha, &smc.EstIalpha, &smc.Zalpha);
    CalcEstI(smc.Vbeta, smc.Ibeta, smc.Ebeta, &smc.EstIbeta, &smc.Zbeta);
    CalcBEMF(&smc.Ealpha, &smc.EalphaFinal, smc.Zalpha);
    CalcBEMF(&smc.Ebeta, &smc.EbetaFinal, smc.Zbeta);
    s->Theta = (Atan2(s->EbetaFinal, smc.EalphaFinal)); // 应该是反正切求出角度，测试使用强托角度
    AccumTheta += (s->Theta - PrevTheta);               // 可能有bug
    PrevTheta = s->Theta;
    AccumThetaCnt++;
    if (AccumThetaCnt == IRP_PERCALC)
    {
        /*******************************************************
          
                                 AccumThetaCnt * 60
                    eRPM = -----------------------------
                                SpeedLoopTime * 65535
        
                                    eRPM * 2
                    RPM = -----------------------------
                                        P
        For example:
        AccumThetaCnt = 16384
        SpeedLoopTime = 0.001
        Then:
        Speed in eRPM is 15000 and RPM is 3000RPM
        
                                                60
        SMO_SPEED_EST_MULTIPLIER = -----------------------------
                                      SpeedLoopTime * 65535      

        ********************************************************/
        s->Omega = (int16_t)(_IQmpy(AccumTheta, SMO_SPEED_EST_MULTIPLIER)); // 电转速
        AccumThetaCnt = 0;
        AccumTheta = 0;
    }
    trans_counter++;
    if (trans_counter == TRANSITION_STEPS)
        trans_counter = 0;
    s->OmegaFltred = s->OmegaFltred + (_IQmpy(s->FiltOmCoef, (s->Omega - s->OmegaFltred)));
    s->Kslf = s->KslfFinal = (int16_t)(_IQmpy(s->OmegaFltred, THETA_FILTER_CNST));
    // 由于滤波器系数是动态的，因此我们需要确保最小
    // 因此我们将最低的运行速度定义为最低的滤波器系数
    Kslf_min = (int16_t)(_IQmpy(ENDSPEED_ELECTR, THETA_FILTER_CNST));
    if (s->Kslf < Kslf_min)
    {
        s->Kslf = Kslf_min;
        s->KslfFinal = s->Kslf;
    }
    s->ThetaOffset = CONSTANT_PHASE_SHIFT;
    s->Theta = s->Theta + s->ThetaOffset;
    return;
}
