#include "smc.h"
#include "MotorConfig.h"
#include "IQmath.h"
#define ONE_BY_SQRT3 0.5773502691
uint16_t trans_counter = 0;
int16_t Theta_error = 0;    //开环强制角度和估算角度的误差,在闭环的过程中慢慢减去误差,每次步进0.05度。
int16_t PrevTheta = 0;      // Previous theta which is then subtracted from Theta to get
                            // delta theta. This delta will be accumulated in AccumTheta, and
                            // after a number of accumulations Omega is calculated.
int16_t AccumTheta = 0;     // Accumulates delta theta over a number of times
uint16_t AccumThetaCnt = 0; // Counter used to calculate motor speed. Is incremented
                            // in SMC_Position_Estimation() subroutine, and accumulates
                            // delta Theta. After N number of accumulations, Omega is
                            // calculated. This N is diIrpPerCalc which is defined in
                            // UserParms.h.
MOTOR_ESTIM_PARM_T motorParm;
SMC smc1 = SMC_DEFAULTS;
ESTIM_PARM_T estimator;

void SMCInit(SMC *s)
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
    s->MaxVoltage = (int16_t)((ADCSample.Voltage * Q15(ONE_BY_SQRT3)) >> 15);
    if (((int32_t)motorParm.qRs << NORM_RS_SCALINGFACTOR) >= ((int32_t)motorParm.qLsDt << NORM_LSDTBASE_SCALINGFACTOR))

        s->Fsmopos = Q15(0.0);
    else
        s->Fsmopos = (0x7FFF - (((int32_t)motorParm.qRs << (15 + NORM_RS_SCALINGFACTOR - NORM_LSDTBASE_SCALINGFACTOR)) / motorParm.qLsDt));

    if (((int32_t)motorParm.qLsDt << NORM_LSDTBASE_SCALINGFACTOR) < 32767)
        s->Gsmopos = 0x7FFF;
    else
        s->Gsmopos = (((int32_t)0x7FFF << (15 - NORM_LSDTBASE_SCALINGFACTOR)) / motorParm.qLsDt);

    s->Kslide = Q15(SMCGAIN);
    s->MaxSMCError = 10;
    s->mdbi = s->Kslide / s->MaxSMCError;
    s->FiltOmCoef = (int16_t)((ENDSPEED_ELECTR * THETA_FILTER_CNST) >> 15);
    return;
}

void CalcBEMF()
{
    int16_t temp_int1, temp_int2;
    temp_int1 = (int16_t)((smc1.Kslf * smc1.Zalpha) >> 15);
    temp_int2 = (int16_t)((smc1.Kslf * smc1.Ealpha) >> 15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.Ealpha = smc1.Ealpha + temp_int1;
    temp_int1 = (int16_t)((smc1.Kslf * smc1.Ealpha) >> 15);
    temp_int2 = (int16_t)((smc1.Kslf * smc1.EalphaFinal) >> 15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.EalphaFinal = smc1.EalphaFinal + temp_int1;

    temp_int1 = (int16_t)((smc1.Kslf * smc1.Zbeta) >> 15);
    temp_int2 = (int16_t)((smc1.Kslf * smc1.Ebeta) >> 15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.Ebeta = smc1.Ebeta + temp_int1;
    temp_int1 = (int16_t)((smc1.Kslf * smc1.Ebeta) >> 15);
    temp_int2 = (int16_t)((smc1.Kslf * smc1.EbetaFinal) >> 15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.EbetaFinal = smc1.EbetaFinal + temp_int1;
}
void CalcEstI()
{
    int16_t temp_int1, temp_int2, temp_int3;
    temp_int1 = (int16_t)((smc1.Gsmopos * smc1.Valpha) >> 15);
    temp_int2 = (int16_t)((smc1.Gsmopos * smc1.Ealpha) >> 23); //原来右移15不行，改23可以
    temp_int3 = (int16_t)((smc1.Gsmopos * smc1.Zalpha) >> 23); //原来右移15不行，改23可以

    temp_int1 = temp_int1 - temp_int2;
    temp_int1 = temp_int1 - temp_int3;
    smc1.EstIalpha = temp_int1 + (int16_t)((smc1.Fsmopos * smc1.EstIalpha) >> 15);

    temp_int1 = (int16_t)((smc1.Gsmopos * smc1.Vbeta) >> 15);
    temp_int2 = (int16_t)((smc1.Gsmopos * smc1.Ebeta) >> 23); //原来右移15不行，改23可以
    temp_int3 = (int16_t)((smc1.Gsmopos * smc1.Zbeta) >> 23); //原来右移15不行，改23可以

    temp_int1 = temp_int1 - temp_int2;
    temp_int1 = temp_int1 - temp_int3;
    smc1.EstIbeta = temp_int1 + (int16_t)((smc1.Fsmopos * smc1.EstIbeta) >> 15);

    smc1.IalphaError = smc1.EstIalpha - smc1.Ialpha;
    smc1.IbetaError = smc1.EstIbeta - smc1.Ibeta;

    if (Abs(smc1.IalphaError) < smc1.MaxSMCError)
    {
        // s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
        smc1.Zalpha = (smc1.mdbi * smc1.IalphaError);
    }
    else if (smc1.IalphaError > 0)
    {
        smc1.Zalpha = smc1.Kslide;
    }
    else
    {
        smc1.Zalpha = -smc1.Kslide;
    }
    if (Abs(smc1.IbetaError) < smc1.MaxSMCError)
    {
        smc1.Zbeta = (smc1.mdbi * smc1.IbetaError);
    }
    else if (smc1.IbetaError > 0)
    {
        smc1.Zbeta = smc1.Kslide;
    }
    else
    {
        smc1.Zbeta = -smc1.Kslide;
    }
}

void SMC_Position_Estimation_Inline(SMC *s)
{
    int16_t Kslf_min;
    CalcEstI();
    CalcBEMF();
    s->Theta = AngleSin_Cos.IQAngle / 360; // 应该是反正切求出角度，测试使用强托角度
    AccumTheta += s->Theta - PrevTheta;
    PrevTheta = s->Theta;
    AccumThetaCnt++;
    if (AccumThetaCnt == IRP_PERCALC)
    {
        //                    AccumThetaCnt * 60
        // eRPM = -----------------------------
        //               SpeedLoopTime * 65535
        //           eRPM * 2
        // RPM = ------------
        //               P
        //        For example:
        //    AccumThetaCnt = 16384
        //    SpeedLoopTime = 0.001
        //Then:
        //    Speed in eRPM is 15000 and RPM is 3000RPM
        //
        //                                   60
        // SMO_SPEED_EST_MULTIPLIER = -------------------------
        //                              SpeedLoopTime * 65535
        s->Omega = (int16_t)((AccumTheta * SMO_SPEED_EST_MULTIPLIER) >> 15);
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
    s->Kslide = Q15(SMCGAIN);
    return;
}
