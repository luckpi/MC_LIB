#include "svgen_dq.h"
#include "IQmath.h"
#include "common.h"
#include "pwm.h"
SVGENDQ SVM = SVGENDQ_DEFAULTS;
void svgendq_calc(void)
{
    int32_t Va, Vb, Vc, t1, t2;
    uint32_t Sector = 0;
    /*
    InvClark：
    Vr1  = Vbeta
    Vr2  = (-Vbeta + √3 * Valpha)/2
    Vr3  = (-Vbeta - √3 * Valpha)/2
    */
    Va = SVM.Vbeta;
    Vb = (-_IQdiv2(SVM.Vbeta)) + _IQmpy(28077, SVM.Valpha); // sqrt(3)/2   _IQ(0.866)
    Vc = (-_IQdiv2(SVM.Vbeta)) - _IQmpy(28077, SVM.Valpha); // sqrt(3)/2   _IQ(0.866)
    // 扇区确定
    if (Va > 0)
        Sector |= 0x01;
    if (Vb > 0)
        Sector |= 0x02;
    if (Vc > 0)
        Sector |= 0x04;

    // if (Sector == 0) // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    // {
    //     SVM.Ta = 16384;
    //     SVM.Tb = 16384;
    //     SVM.Tc = 16384;
    // }
    if (Sector == 1) // 60-120
    {
        t1 = -Vb;
        t2 = -Vc;
        SVM.Tc = _IQdiv2(32767 - t1 - t2);
        SVM.Ta = SVM.Tc + t2;
        SVM.Tb = SVM.Ta + t1;
    }
    else if (Sector == 2) // 300-360
    {
        t1 = -Vc;
        t2 = -Va;
        SVM.Tb = _IQdiv2(32767 - t1 - t2);
        SVM.Tc = SVM.Tb + t2;
        SVM.Ta = SVM.Tc + t1;
    }
    else if (Sector == 3) // 0-60
    {
        t1 = Vb;
        t2 = Va;
        SVM.Tc = _IQdiv2(32767 - t1 - t2);
        SVM.Tb = SVM.Tc + t2;
        SVM.Ta = SVM.Tb + t1;
    }
    else if (Sector == 4) // 180-240
    {
        t1 = -Va;
        t2 = -Vb;
        SVM.Ta = _IQdiv2(32767 - t1 - t2);
        SVM.Tb = SVM.Ta + t2;
        SVM.Tc = SVM.Tb + t1;
    }
    else if (Sector == 5) // 120-180
    {
        t1 = Va;
        t2 = Vc;
        SVM.Ta = _IQdiv2(32767 - t1 - t2);
        SVM.Tc = SVM.Ta + t2;
        SVM.Tb = SVM.Tc + t1;
    }
    else if (Sector == 6) // 240-300
    {
        t1 = Vc;
        t2 = Vb;
        SVM.Tb = _IQdiv2(32767 - t1 - t2);
        SVM.Ta = SVM.Tb + t2;
        SVM.Tc = SVM.Ta + t1;
    }
}
