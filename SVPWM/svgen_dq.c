#include "svgen_dq.h"
#include "IQmath.h"
#include "common.h"
#include "pwm.h"
SVGENDQ SVP = SVGENDQ_DEFAULTS;
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
    Va = SVP.Vbeta;
    Vb = (-_IQdiv2(SVP.Vbeta)) + _IQmpy(28077, SVP.Valpha); // sqrt(3)/2   _IQ(0.866)
    Vc = (-_IQdiv2(SVP.Vbeta)) - _IQmpy(28077, SVP.Valpha); // sqrt(3)/2   _IQ(0.866)
    // 扇区确定
    if (Va > 0)
        Sector = 1;
    if (Vb > 0)
        Sector += 2;
    if (Vc > 0)
        Sector += 4;
    // 三相计算
    Va = SVP.Vbeta;
    Vb = _IQdiv2(SVP.Vbeta) + _IQmpy(28077, SVP.Valpha); // sqrt(3)/2
    Vc = _IQdiv2(SVP.Vbeta) - _IQmpy(28077, SVP.Valpha); // sqrt(3)/2

    if (Sector == 0) // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    {
        SVP.Ta = 16384;
        SVP.Tb = 16384;
        SVP.Tc = 16384;
    }
    else if (Sector == 1) // Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)，60-120
    {
        t1 = Vc;
        t2 = Vb;
        SVP.Tb = _IQmpy((32768 - t1 - t2), 16384); // tbon = (1-t1-t2)/2
        SVP.Ta = SVP.Tb + t1;                      // taon = tbon+t1
        SVP.Tc = SVP.Ta + t2;                      // tcon = taon+t2
    }
    else if (Sector == 2) //Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)，300-360
    {
        t1 = Vb;
        t2 = -Va;
        SVP.Ta = _IQmpy((32768 - t1 - t2), 16384); // taon = (1-t1-t2)/2
        SVP.Tc = SVP.Ta + t1;                      // tcon = taon+t1
        SVP.Tb = SVP.Tc + t2;                      // tbon = tcon+t2
    }
    else if (Sector == 3)
    // Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)，0-60
    {
        t1 = -Vc;
        t2 = Va;
        SVP.Ta = _IQmpy((32768 - t1 - t2), 16384); // taon = (1-t1-t2)/2
        SVP.Tb = SVP.Ta + t1;                      // tbon = taon+t1
        SVP.Tc = SVP.Tb + t2;                      // tcon = tbon+t2
    }
    else if (Sector == 4)
    // Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)，180-240
    {
        t1 = -Va;
        t2 = Vc;
        SVP.Tc = _IQmpy((32768 - t1 - t2), 16384); // tcon = (1-t1-t2)/2
        SVP.Tb = SVP.Tc + t1;                      // tbon = tcon+t1
        SVP.Ta = SVP.Tb + t2;                      // taon = tbon+t2
    }
    else if (Sector == 5) // Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)，120-180
    {
        t1 = Va;
        t2 = -Vb;
        SVP.Tb = _IQmpy((32768 - t1 - t2), 16384); // tbon = (1-t1-t2)/2
        SVP.Tc = SVP.Tb + t1;                      // tcon = tbon+t1
        SVP.Ta = SVP.Tc + t2;                      // taon = tcon+t2
    }
    else if (Sector == 6) // Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)，240-300
    {
        t1 = -Vb;
        t2 = -Vc;
        SVP.Tc = _IQmpy((32768 - t1 - t2), 16384); // tcon = (1-t1-t2)/2
        SVP.Ta = SVP.Tc + t1;                      // taon = tcon+t1
        SVP.Tb = SVP.Ta + t2;                      // tbon = taon+t2
    }
    // // Convert the unsigned GLOBAL_Q format (ranged (0,1)) -> signed GLOBAL_Q format (ranged(-1,1))
    // SVP.Ta = (2.0 * (SVP.Ta - 0.5));
    // SVP.Tb = (2.0 * (SVP.Tb - 0.5));
    // SVP.Tc = (2.0 * (SVP.Tc - 0.5));
    // printf("Ta=%d,Tb=%d,Tc=%d\n", SVP.Ta, SVP.Tb, SVP.Tc);
    // printf("%d", Sector);
}
