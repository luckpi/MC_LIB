#include "svgen_dq.h"
#include "IQmath.h"
#include "common.h"
#include "pwm.h"
SVGENDQ SVM = SVGENDQ_DEFAULTS;
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
    Vb = (-_IQdiv2(s->Vbeta)) + _IQmpy(28077, s->Valpha); // sqrt(3)/2   _IQ(0.866)
    Vc = (-_IQdiv2(s->Vbeta)) - _IQmpy(28077, s->Valpha); // sqrt(3)/2   _IQ(0.866)
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
        t1 = -Vb;
        t2 = -Vc;
        s->Tc = _IQdiv2(32767 - t1 - t2);
        s->Ta = s->Tc + t2;
        s->Tb = s->Ta + t1;
    }
    else if (Sector == 2) // 300-360
    {
        t1 = -Vc;
        t2 = -Va;
        s->Tb = _IQdiv2(32767 - t1 - t2);
        s->Tc = s->Tb + t2;
        s->Ta = s->Tc + t1;
    }
    else if (Sector == 3) // 0-60
    {
        t1 = Vb;
        t2 = Va;
        s->Tc = _IQdiv2(32767 - t1 - t2);
        s->Tb = s->Tc + t2;
        s->Ta = s->Tb + t1;
    }
    else if (Sector == 4) // 180-240
    {
        t1 = -Va;
        t2 = -Vb;
        s->Ta = _IQdiv2(32767 - t1 - t2);
        s->Tb = s->Ta + t2;
        s->Tc = s->Tb + t1;
    }
    else if (Sector == 5) // 120-180
    {
        t1 = Va;
        t2 = Vc;
        s->Ta = _IQdiv2(32767 - t1 - t2);
        s->Tc = s->Ta + t2;
        s->Tb = s->Tc + t1;
    }
    else if (Sector == 6) // 240-300
    {
        t1 = Vc;
        t2 = Vb;
        s->Tb = _IQdiv2(32767 - t1 - t2);
        s->Ta = s->Tb + t2;
        s->Tc = s->Ta + t1;
    }
}
