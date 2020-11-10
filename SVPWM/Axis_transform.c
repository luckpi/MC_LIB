#include "common.h"
#include "IQmath.h"
#include "svgen_dq.h"
//α：Alpha β：Beta θ：Theta
//  1/sqrt(3) = 0.57735026918963
#define ONEbySQRT3 0.57735026918963 /* 1/sqrt(3) */
/*
Clark：
    ia + ib + ic = 0 ;
    iAlpha = ia ;
    iBeta = (ia + 2 * ib) / √3;
*/
void Clark_Cala(void)
{
    // SVP.Ic = 0 - (SVP.Ia + SVP.Ib);
    SVP.Ialpha = SVP.Ia;
    SVP.Ibeta = _IQmpy((SVP.Ia + _IQmpy2(SVP.Ib)), 18918); // _IQ(0.57735026918963)
}
/*
Park：
    ld =  iAlpha * cos(θ) + iBeta * cos(θ)
    lq = -iAlpha * sin(θ) + iBeta * cos(θ)
*/
void Park_Cala(void)
{
    IQSin_Cos_Cale((p_IQSin_Cos)&AngleSin_Cos);
    SVP.Sine = AngleSin_Cos.IQSin;
    SVP.Cosine = AngleSin_Cos.IQCos;
    SVP.Ld = _IQmpy(SVP.Ialpha, SVP.Cosine) + _IQmpy(SVP.Ibeta, SVP.Sine);
    SVP.Lq = _IQmpy(SVP.Ibeta, SVP.Cosine) - _IQmpy(SVP.Ialpha, SVP.Sine);
}
/*
InvPark：
    Valpha = vd * cos(θ) - vq * sin(θ)
    Vbeta  = vd * sin(θ) + vq * cos(θ)
*/
void InvPark(void)
{
    IQSin_Cos_Cale((p_IQSin_Cos)&AngleSin_Cos);
    SVP.Sine = AngleSin_Cos.IQSin;
    SVP.Cosine = AngleSin_Cos.IQCos;
    SVP.Valpha = _IQmpy(SVP.Vd, SVP.Cosine) - _IQmpy(SVP.Vq, SVP.Sine);
    SVP.Vbeta = _IQmpy(SVP.Vd, SVP.Sine) + _IQmpy(SVP.Vq, SVP.Cosine);
}


