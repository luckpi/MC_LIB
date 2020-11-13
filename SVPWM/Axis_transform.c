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
    // SVM.Ic = 0 - (SVM.Ia + SVM.Ib);
    SVM.Ialpha = SVM.Ia;
    SVM.Ibeta = _IQmpy((SVM.Ia + (SVM.Ib << 1)), 18918); // _IQ(0.57735026918963)
}
/*
Park：
    ld =  iAlpha * cos(θ) + iBeta * cos(θ)
    lq = -iAlpha * sin(θ) + iBeta * cos(θ)
*/
void Park_Cala(void)
{
    IQSin_Cos_Cale((p_IQSin_Cos)&AngleSin_Cos);
    SVM.Sine = AngleSin_Cos.IQSin;
    SVM.Cosine = AngleSin_Cos.IQCos;
    SVM.Ld = _IQmpy(SVM.Ialpha, SVM.Cosine) + _IQmpy(SVM.Ibeta, SVM.Sine);
    SVM.Lq = _IQmpy(SVM.Ibeta, SVM.Cosine) - _IQmpy(SVM.Ialpha, SVM.Sine);
}
/*
InvPark：
    Valpha = vd * cos(θ) - vq * sin(θ)
    Vbeta  = vd * sin(θ) + vq * cos(θ)
*/
void InvPark(void)
{
    IQSin_Cos_Cale((p_IQSin_Cos)&AngleSin_Cos);
    SVM.Sine = AngleSin_Cos.IQSin;
    SVM.Cosine = AngleSin_Cos.IQCos;
    SVM.Valpha = _IQmpy(SVM.Vd, SVM.Cosine) - _IQmpy(SVM.Vq, SVM.Sine);
    SVM.Vbeta = _IQmpy(SVM.Vd, SVM.Sine) + _IQmpy(SVM.Vq, SVM.Cosine);
}
