#include "smc.h"

int16_t PrevTheta = 0;      // 上一次角度值
int16_t AccumTheta = 0;     // 累加角度变化量
int16_t Theta_error = 0;    // 开环角和估算角的误差
uint16_t trans_counter = 0; // 减小开环角和估算角差距间隔
uint16_t AccumThetaCnt = 0; // 用于计算电机角速度的频率计数器
SMC smc = SMC_DEFAULTS;
MOTOR_ESTIM motorParm;

/*****************************************************************************
 函 数 名  : SMC_Init
 功能描述  : 滑膜控制器参数初始化
 输入参数  : 滑膜参数结构体地址
 输出参数  : void
*****************************************************************************/
void SMC_Init(p_SMC s, p_MOTOR_ESTIM m)
{
    // 电机参数归一化
    m->Vol_Const = MAX_MOTOR_VOLTAGE * ONE_BY_SQRT3 / 32768.0 * (1.0 - PWM_DTS / PWM_TS);
    m->Cur_Const = MAX_MOTOR_CURRENT / 32768.0;
    m->Omg_Const = TWO_PI / 60.0;
    m->qLsDtBase = (Q15(MOTOR_LS / m->Vol_Const * m->Cur_Const / PWM_TS)) >> 8;
    m->qLsDt = m->qLsDtBase;
    m->qRs = (Q15(MOTOR_RS / m->Vol_Const * m->Cur_Const)) >> 1;
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //             Ts
    // Gsmopos = ------
    //              L
    // Ts = 采样周期。 如果以PWM采样, Ts = 62.5 us
    // R  = 相位电阻。 如果电机数据表未提供，用万用表测量相位电阻,除以二得到相位电阻
    // L  = 相位电感。 如果电机数据表未提供，用万用表测量相位电感,除以二得到相位电感
    if (((int32_t)m->qRs << NORM_RS_SCALINGFACTOR) >= ((int32_t)m->qLsDt << NORM_LSDTBASE_SCALINGFACTOR))

        s->Fsmopos = 0;
    else
        s->Fsmopos = (0x7FFF - (((int32_t)m->qRs << (15 + NORM_RS_SCALINGFACTOR - NORM_LSDTBASE_SCALINGFACTOR)) / m->qLsDt));

    if (((int32_t)m->qLsDt << NORM_LSDTBASE_SCALINGFACTOR) < 32767)
        s->Gsmopos = 0x7FFF;
    else
        s->Gsmopos = ((int32_t)0x7FFF << (15 - NORM_LSDTBASE_SCALINGFACTOR)) / m->qLsDt;

    s->Kslide = Q15(SMCGAIN);
    s->MaxSMCError = Q15(MAXLINEARSMC);
    s->mdbi = s->Kslide / s->MaxSMCError;
    s->Kslf_min = _IQmpy(ENDSPEED_ELECTR, THETA_FILTER_CNST);
    s->FiltOmCoef = _IQmpy(ENDSPEED_ELECTR, THETA_FILTER_CNST);
    s->ThetaOffset = CONSTANT_PHASE_SHIFT;
    // s->MaxVoltage = (int16_t)(_IQmpy(ADCSample.Voltage, 18918));//_IQ(0.57735026918963) 最大矢量电压
    return;
}

/*****************************************************************************
 函 数 名  : LPF_Filter
 功能描述  : 自适应滤波器
 输入参数  : Kslf ,I, O
 输出参数  : void
*****************************************************************************/
void LPF_Filter(int16_t Kslf, int16_t In, int16_t *Out)
{
    // Kslf ：滑动模式控制器低通滤波器的增益     eRPS：电机的电气转速，单位为 RPS
    // Kslf = PWM_Ts * 2pi * eRPS
    // Out = Out + (Kslf * (In - Out))
    (*Out) += _IQmpy(Kslf, (In - (*Out)));
}

/*****************************************************************************
 函 数 名  : CalcBEMF
 功能描述  : 估算反电动势滤波
 输入参数  : *EMF , *EMFF,  Z
 输出参数  : void
*****************************************************************************/
void CalcBEMF(p_SMC s)
{
    // α轴反电动势
    LPF_Filter(s->Kslf, s->Zalpha, &s->Ealpha);      // 滤波用来计算下一个估算电流
    LPF_Filter(s->Kslf, s->Ealpha, &s->EalphaFinal); // 滤波用来计算估算角

    // β轴反电动势
    LPF_Filter(s->Kslf, s->Zbeta, &s->Ebeta);      // 滤波用来计算下一个估算电流
    LPF_Filter(s->Kslf, s->Ebeta, &s->EbetaFinal); // 滤波用来计算估算角
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
U   ：Vbus / √3 * Valpha(Vbeta)  这里是百分比
EMF： 估算的反电动势
EstI：估算的电流
z   : 校准因子
*/
void CalcEstI(p_SMC s, int16_t U, int16_t I, int16_t EMF, int16_t *EstI, int16_t *Z)
{
    int16_t I_Error;
    *EstI = _IQmpy(s->Fsmopos, (*EstI)) + _IQmpy(s->Gsmopos, (U - EMF - (*Z)));
    I_Error = *EstI - I;
    if (I_Error > s->MaxSMCError)
    {
        *Z = s->Kslide;
    }
    else if (I_Error < -s->MaxSMCError)
    {
        *Z = -s->Kslide;
    }
    else
    {
        // s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
        *Z = _IQmpy(s->mdbi, I_Error);
    }
}

/*****************************************************************************
 函 数 名  : SMC_Position_Estimation
 功能描述  : 滑膜控制器，估算角度
 输入参数  : 滑膜参数结构体地址
 输出参数  : void
*****************************************************************************/
void SMC_Position_Estimation(p_SMC s)
{
    CalcEstI(s, s->Valpha, s->Ialpha, s->Ealpha, &s->EstIalpha, &s->Zalpha);
    CalcEstI(s, s->Vbeta, s->Ibeta, s->Ebeta, &s->EstIbeta, &s->Zbeta);
    CalcBEMF(s);
    s->Theta = CORDIC_Atan(s->EbetaFinal, -s->EalphaFinal); // 应该是反正切求出角度，测试使用强托角度
    AccumTheta += s->Theta - PrevTheta;                     // 可能有bug
    PrevTheta = s->Theta;
    if (++AccumThetaCnt == IRP_PERCALC)
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
        s->Omega = _IQmpy(AccumTheta, SMO_SPEED_EST_MULTIPLIER); // 电转速
        AccumThetaCnt = 0;
        AccumTheta = 0;
    }
    if (++trans_counter == TRANSITION_STEPS)
        trans_counter = 0;
    s->OmegaFltred += _IQmpy(s->FiltOmCoef, (s->Omega - s->OmegaFltred));
    s->Kslf = _IQmpy(s->OmegaFltred, THETA_FILTER_CNST) * HoldParm.RotorDirection;
    // 由于滤波器系数是动态的，因此我们需要确保最小
    // 因此我们将最低的运行速度定义为最低的滤波器系数
    if (s->Kslf < s->Kslf_min)
    {
        s->Kslf = s->Kslf_min;
    }
    s->Theta += s->ThetaOffset;
    return;
}
