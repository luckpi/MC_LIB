#include "smc.h"
#include "MotorConfig.h"
#include "IQmath.h"
#include "atan2.h"
#define ONE_BY_SQRT3 0.5773502691
uint16_t trans_counter = 0;
int16_t Theta_error = 0;    // 开环强制角度和估算角度的误差,在闭环的过程中慢慢减去误差,每次步进0.05度。
uint16_t PrevTheta = 0;      // 上一次角度值
int16_t AccumTheta = 0;     // 累加每次的角度变化量
uint16_t AccumThetaCnt = 0; // 用于计算电机速度的计数器.
MOTOR_ESTIM_PARM_T motorParm;
SMC smc = SMC_DEFAULTS;

void SMC_Init(SMC *s)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = 采样周期。 如果以PWM采样, Ts = 50 us
    // R  = 相位电阻。 如果电机数据表未提供，用万用表测量相位电阻,除以二得到相位电阻
    // L  = 相位电感。 如果电机数据表未提供，用万用表测量相位电感,除以二得到相位电感

    motorParm.qLsDtBase = NORM_LSDTBASE;
    motorParm.qLsDt = motorParm.qLsDtBase;
    motorParm.qRs = NORM_RS;
    if (((int32_t)motorParm.qRs << NORM_RS_SCALINGFACTOR) >= ((int32_t)motorParm.qLsDt << NORM_LSDTBASE_SCALINGFACTOR))

        s->Fsmopos = Q15(0.0);
    else
        s->Fsmopos = (0x7FFF - HDIV_div(((int32_t)motorParm.qRs << (15 + NORM_RS_SCALINGFACTOR - NORM_LSDTBASE_SCALINGFACTOR)), motorParm.qLsDt));

    if (((int32_t)motorParm.qLsDt << NORM_LSDTBASE_SCALINGFACTOR) < 32767)
        s->Gsmopos = 0x7FFF;
    else
        s->Gsmopos = HDIV_div(((int32_t)0x7FFF << (15 - NORM_LSDTBASE_SCALINGFACTOR)), motorParm.qLsDt);

    s->Kslide = Q15(SMCGAIN);
    s->MaxSMCError = Q15(MAXLINEARSMC);
    s->mdbi = HDIV_div(s->Kslide, s->MaxSMCError);
    s->FiltOmCoef = (int16_t)((ENDSPEED_ELECTR * THETA_FILTER_CNST) >> 15);
    s->MaxVoltage = (int16_t)((ADCSample.Voltage * Q15(ONE_BY_SQRT3)) >> 15);
    return;
}

void CalcBEMF(int16_t *EMF, int16_t *EMFF, int16_t Z)
{
    int16_t temp_int1, temp_int2;
    temp_int1 = (int16_t)((smc.Kslf * Z) >> 15);
    temp_int2 = (int16_t)((smc.Kslf * (*EMF)) >> 15);
    temp_int1 -= temp_int2;
    (*EMF) += temp_int1;
    temp_int1 = (int16_t)((smc.Kslf * (*EMF)) >> 15);
    temp_int2 = (int16_t)((smc.Kslf * (*EMFF)) >> 15);
    temp_int1 -= temp_int2;
    (*EMFF) += temp_int1;
}
void CalcEstI(int16_t U, int16_t I, int16_t EMF, int16_t *EstI, int16_t *Z)
{
    int16_t temp_int1, temp_int2, temp_int3, I_Error;
    temp_int1 = (int16_t)((smc.Gsmopos * U) >> 15);
    temp_int2 = (int16_t)((smc.Gsmopos * EMF) >> 15);  //原来右移15不行，改23可以
    temp_int3 = (int16_t)((smc.Gsmopos * (*Z)) >> 15); //原来右移15不行，改23可以
    temp_int1 -= temp_int2;
    temp_int1 -= temp_int3;
    *EstI = temp_int1 + (int16_t)((smc.Fsmopos * (*EstI)) >> 15);
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
        s->Omega = (int16_t)((AccumTheta * SMO_SPEED_EST_MULTIPLIER) >> 15); // 电转速
        AccumThetaCnt = 0;
        AccumTheta = 0;
    }
    trans_counter++;
    if (trans_counter == TRANSITION_STEPS)
        trans_counter = 0;
    s->OmegaFltred = s->OmegaFltred + ((s->FiltOmCoef * (s->Omega - s->OmegaFltred)) >> 15);
    s->Kslf = s->KslfFinal = (int16_t)((s->OmegaFltred * THETA_FILTER_CNST) >> 15);
    // 由于滤波器系数是动态的，因此我们需要确保最小
    // 因此我们将最低的运行速度定义为最低的滤波器系数
    Kslf_min = (int16_t)((ENDSPEED_ELECTR * THETA_FILTER_CNST) >> 15);
    if (s->Kslf < Kslf_min)
    {
        s->Kslf = Kslf_min;
        s->KslfFinal = s->Kslf;
    }
    s->ThetaOffset = CONSTANT_PHASE_SHIFT;
    s->Theta = s->Theta + s->ThetaOffset;
    return;
}
