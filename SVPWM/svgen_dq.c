#include "MotorConfig.h"
#include "svgen_dq.h"
#include "IQmath.h"
#include "common.h"
#include "pwm.h"
SVGENDQ svm = SVGENDQ_DEFAULTS;
/*****************************************************************************
 函 数 名  : svgendq_Init
 功能描述  : 初始化SVPWM参数
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void svgendq_Init(p_SVGENDQ s)
{
    s->Ia = 0;
    s->Ib = 0;
    s->Ic = 0;
    s->Ia_C = 0;
    s->Ib_C = 0;
    s->Ialpha = 0;
    s->Ibeta = 0;
    s->Theta = 0;
    s->Sine = 0;
    s->Cosine = 0;
    s->Id = 0;
    s->Iq = 0;
    s->Vd = 0;
    s->Vq = 0;
    s->Valpha = 0;
    s->Vbeta = 0;
    s->Ta = 0;
    s->Tb = 0;
    s->Tc = 0;
}
/*****************************************************************************
 函 数 名  : svgendq_calc
 功能描述  : SVPWM计算
 输入参数  : SVM结构体地址
 输出参数  : void
*****************************************************************************/
void svgendq_calc(p_SVGENDQ s)
{
    int32_t Va, Vb, Vc, t1, t2;
    uint8_t Sector = 0;
    /*
    InvClark：
    Vr1  = Vbeta
    Vr2  = (-Vbeta + √3 * Valpha)/2
    Vr3  = (-Vbeta - √3 * Valpha)/2
    */
    Va = s->Vbeta;
    Vb = (-_IQdiv2(s->Vbeta)) + _IQmpy(28377, s->Valpha); // sqrt(3)/2   _IQ(0.866)
    Vc = (-_IQdiv2(s->Vbeta)) - _IQmpy(28377, s->Valpha); // sqrt(3)/2   _IQ(0.866)
    // 扇区确定
    if (Va > 0)
        Sector |= 0x01;
    if (Vb > 0)
        Sector |= 0x02;
    if (Vc > 0)
        Sector |= 0x04;

    // if (Sector == 0) // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    // {
    //     s->Ta = 16384;
    //     s->Tb = 16384;
    //     s->Tc = 16384;
    // }
    if (Sector == 1) // 60-120
    {
        t1 = -_IQmpy(Vb, PWM_FRE_SETATA);
        t2 = -_IQmpy(Vc, PWM_FRE_SETATA);
        s->Tc = _IQdiv2(PWM_FRE_SETATA - t1 - t2);
        s->Ta = s->Tc + t2;
        s->Tb = s->Ta + t1;
    }
    else if (Sector == 2) // 300-360
    {
        t1 = -_IQmpy(Vc, PWM_FRE_SETATA);
        t2 = -_IQmpy(Va, PWM_FRE_SETATA);
        s->Tb = _IQdiv2(PWM_FRE_SETATA - t1 - t2);
        s->Tc = s->Tb + t2;
        s->Ta = s->Tc + t1;
    }
    else if (Sector == 3) // 0-60
    {
        t1 = _IQmpy(Vb, PWM_FRE_SETATA);
        t2 = _IQmpy(Va, PWM_FRE_SETATA);
        s->Tc = _IQdiv2(PWM_FRE_SETATA - t1 - t2);
        s->Tb = s->Tc + t2;
        s->Ta = s->Tb + t1;
    }
    else if (Sector == 4) // 180-240
    {
        t1 = -_IQmpy(Va, PWM_FRE_SETATA);
        t2 = -_IQmpy(Vb, PWM_FRE_SETATA);
        s->Ta = _IQdiv2(PWM_FRE_SETATA - t1 - t2);
        s->Tb = s->Ta + t2;
        s->Tc = s->Tb + t1;
    }
    else if (Sector == 5) // 120-180
    {
        t1 = _IQmpy(Va, PWM_FRE_SETATA);
        t2 = _IQmpy(Vc, PWM_FRE_SETATA);
        s->Ta = _IQdiv2(PWM_FRE_SETATA - t1 - t2);
        s->Tc = s->Ta + t2;
        s->Tb = s->Tc + t1;
    }
    else if (Sector == 6) // 240-300
    {
        t1 = _IQmpy(Vc, PWM_FRE_SETATA);
        t2 = _IQmpy(Vb, PWM_FRE_SETATA);
        s->Tb = _IQdiv2(PWM_FRE_SETATA - t1 - t2);
        s->Ta = s->Tb + t2;
        s->Tc = s->Ta + t1;
    }
}
