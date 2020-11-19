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
    if (Sector == 1) // Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)，60-120
    {
        t1 = -Vb;
        t2 = -Vc;
        SVM.Tc = _IQdiv2(32767 - t1 - t2); // tbon = (1-t1-t2)/2
        SVM.Ta = SVM.Tc + t2;              // taon = tbon+t1
        SVM.Tb = SVM.Ta + t1;              // tcon = taon+t2
    }
    else if (Sector == 2) //Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)，300-360
    {
        t1 = -Vc;
        t2 = -Va;
        SVM.Tb = _IQdiv2(32767 - t1 - t2); // taon = (1-t1-t2)/2
        SVM.Tc = SVM.Tb + t2;              // tcon = taon+t1
        SVM.Ta = SVM.Tc + t1;              // tbon = tcon+t2
    }
    else if (Sector == 3)
    // Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)，0-60
    {
        t1 = Vb;
        t2 = Va;
        SVM.Tc = _IQdiv2(32767 - t1 - t2); // taon = (1-t1-t2)/2
        SVM.Tb = SVM.Tc + t2;              // tbon = taon+t1
        SVM.Ta = SVM.Tb + t1;              // tcon = tbon+t2
    }
    else if (Sector == 4)
    // Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)，180-240
    {
        t1 = -Va;
        t2 = -Vb;
        SVM.Ta = _IQdiv2(32767 - t1 - t2); // tcon = (1-t1-t2)/2
        SVM.Tb = SVM.Ta + t2;              // tbon = tcon+t1
        SVM.Tc = SVM.Tb + t1;              // taon = tbon+t2
    }
    else if (Sector == 5) // Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)，120-180
    {
        t1 = Va;
        t2 = Vc;
        SVM.Ta = _IQdiv2(32767 - t1 - t2); // tbon = (1-t1-t2)/2
        SVM.Tc = SVM.Ta + t2;              // tcon = tbon+t1
        SVM.Tb = SVM.Tc + t1;              // taon = tcon+t2
    }
    else if (Sector == 6) // Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)，240-300
    {
        t1 = Vc;
        t2 = Vb;
        SVM.Tb = _IQdiv2(32767 - t1 - t2); // tcon = (1-t1-t2)/2
        SVM.Ta = SVM.Tb + t2;              // taon = tcon+t1
        SVM.Tc = SVM.Ta + t1;              // tbon = taon+t2
    }
}
