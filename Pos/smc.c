#include "smc.h"

MOTOR_ESTIM motorParm;
SMC smc = SMC_DEFAULTS;
int16_t PrevTheta = 0;      // 上一次角度值
int16_t AccumTheta = 0;     // 累加角度变化量
int16_t Theta_error = 0;    // 开环角和估算角的误差
uint16_t trans_counter = 0; // 减小开环角和估算角差距间隔
uint16_t AccumThetaCnt = 0; // 用于计算电机角速度的频率计数器

/*****************************************************************************
 函 数 名  : SMC_Init
 功能描述  : 滑膜控制器参数初始化
 输入参数  : 滑膜参数结构体地址
 输出参数  : void
*****************************************************************************/
void SMC_Init(p_SMC s, p_MOTOR_ESTIM m)
{
    // 电机参数归一化, 通过计算，将电压和电流归一化处理
    m->Cur_Vol = MAX_MOTOR_CURRENT / (MAX_MOTOR_VOLTAGE * ONE_BY_SQRT3 * (1.0 - PWM_DTS / PWM_TS)) * 32767;
    m->qLsDt = MOTOR_LS * 32767 / PWM_TS;
    m->qRs = MOTOR_RS * m->Cur_Vol;
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //             Ts
    // Gsmopos = ------
    //              L
    // Ts = 采样周期。 如果以PWM采样, Ts = PWM_Ts
    // R  = 相位电阻。 如果电机数据表未提供，用万用表测量线电阻,除以二得到相位电阻
    // L  = 相位电感。 如果电机数据表未提供，用万用表测量线电感,除以二得到相位电感
    if (m->qRs >= m->qLsDt)
        s->Fsmopos = 0;
    else
        s->Fsmopos = 32767 - _IQdiv(m->qRs, m->qLsDt);

    if (m->qLsDt < 32767)
        s->Gsmopos = 32767;
    else
        s->Gsmopos = _IQdiv(32767, m->qLsDt);

    s->Kslide = Q15(SMCGAIN);
    s->MaxSMCError = Q15(MAXLINEARSMC);
    // Kslide / MaxSMCError
    s->mdbi = Q15((SMCGAIN / MAXLINEARSMC));
    // BEMF动态滤波器系数最小值
    s->Kslf_min = _IQmpy((ENDSPEED_ELECTR * 65535 / 60 / SPEEDLOOPFREQ), THETA_FILTER_CNST);
    // 转子角速度滤波器系数
    s->FiltOmCoef = s->Kslf_min;
    // 相位补偿，经过两次滤波后的BEMF存在90°的相位延时
    s->ThetaOffset = CONSTANT_PHASE_SHIFT;

    // 其他参数初始化
    s->Ealpha = 0;
    s->Zalpha = 0;
    s->EstIalpha = 0;
    s->EalphaFinal = 0;
    s->Ebeta = 0;
    s->Zbeta = 0;
    s->EstIbeta = 0;
    s->EbetaFinal = 0;
    s->Theta = 0;
    s->Omega = 0;
    s->OmegaFltred = 0;
    s->Kslf = s->Kslf_min;

    return;
}

/*****************************************************************************
 函 数 名  : CalcBEMF
 功能描述  : 估算反电动势滤波
 输入参数  : *EMF , *EMFF,  Z
 输出参数  : void
*****************************************************************************/
void CalcBEMF(p_SMC s)
{

    // Out += Kslf * (In - Out)

    // α轴反电动势
    s->Ealpha += _IQmpy(s->Kslf, (s->Zalpha - s->Ealpha));           // 滤波用来计算下一个估算电流
    s->EalphaFinal += _IQmpy(s->Kslf, (s->Ealpha - s->EalphaFinal)); // 滤波用来计算估算角

    // β轴反电动势
    s->Ebeta += _IQmpy(s->Kslf, (s->Zbeta - s->Ebeta));           // 滤波用来计算下一个估算电流
    s->EbetaFinal += _IQmpy(s->Kslf, (s->Ebeta - s->EbetaFinal)); // 滤波用来计算估算角
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
U   ：Vbus / √3 * Valpha(Vbeta)  归一化处理时，将其耦合在F增益中
EMF： 估算的反电动势
EstI：估算的电流
Z   : 校准因子
*/
void CalcEstI(p_SMC s, p_SVGENDQ m)
{
    // 估算α轴电流
    s->EstIalpha = _IQmpy(s->Fsmopos, s->EstIalpha) + _IQmpy(s->Gsmopos, (m->Valpha - s->Ealpha - s->Zalpha));
    s->IalphaError = s->EstIalpha - m->Ialpha;
    if (s->IalphaError > s->MaxSMCError)
    {
        s->Zalpha = s->Kslide;
    }
    else if (s->IalphaError < -s->MaxSMCError)
    {
        s->Zalpha = -s->Kslide;
    }
    else
    {
        // s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
        s->Zalpha = _IQmpy(s->mdbi, s->IalphaError);
    }

    // 估算β轴电流
    s->EstIbeta = _IQmpy(s->Fsmopos, s->EstIbeta) + _IQmpy(s->Gsmopos, (m->Vbeta - s->Ebeta - s->Zbeta));
    s->IbetaError = s->EstIbeta - m->Ibeta;
    if (s->IbetaError > s->MaxSMCError)
    {
        s->Zbeta = s->Kslide;
    }
    else if (s->IbetaError < -s->MaxSMCError)
    {
        s->Zbeta = -s->Kslide;
    }
    else
    {
        // s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
        s->Zbeta = _IQmpy(s->mdbi, s->IbetaError);
    }
}

/*****************************************************************************
 函 数 名  : SMC_Position_Estimation
 功能描述  : 滑膜控制器，估算角度
 输入参数  : 滑膜参数结构体地址
 输出参数  : void
*****************************************************************************/
void SMC_Position_Estimation(p_SMC s, p_SVGENDQ m)
{
    CalcEstI(s, m);
    CalcBEMF(s);
    s->Theta = CORDIC_Atan(s->EbetaFinal, -s->EalphaFinal); // 反正切求角度
    AccumTheta += s->Theta - PrevTheta;                     // 累加固定周期内的角度值用于速度计算
    PrevTheta = s->Theta;
    if (++AccumThetaCnt == IRP_PERCALC)
    {
        s->Omega = AccumTheta; // eRPS = (s->Omega / 65535) / SpeedLoopTime
        AccumThetaCnt = 0;
        AccumTheta = 0;
    }
    if (++trans_counter == TRANSITION_STEPS)
        trans_counter = 0;
    s->OmegaFltred += _IQmpy(s->FiltOmCoef, (s->Omega - s->OmegaFltred));
    // Kslf = PWM_Ts * 2 * PI * eRPS
    s->Kslf = _IQmpy(s->OmegaFltred, THETA_FILTER_CNST);
    // 动态低通滤波器系数限幅
    if (s->Kslf < s->Kslf_min)
    {
        s->Kslf = s->Kslf_min;
    }
    s->Theta += s->ThetaOffset;
    return;
}
