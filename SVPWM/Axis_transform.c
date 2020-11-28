#include "common.h"
#include "IQmath.h"
#include "svgen_dq.h"
//α：Alpha β：Beta θ：Theta
//  1/sqrt(3) = 0.57735026918963

// 电流需要在下管导通中间时刻采集！！！！！！！！！

/*
Clark：
    ia + ib + ic = 0 ;
    iAlpha = ia ;
    iBeta = (ia + 2 * ib) / √3;
*/
void Clark_Cala(p_SVGENDQ s)
{
    s->Ialpha = s->Ia;
    s->Ibeta = _IQmpy((s->Ia + _IQmpy2(s->Ib)), 18918); // _IQ(0.57735026918963)
}
/*
Park：
    ld =  iAlpha * cos(θ) + iBeta * sin(θ)
    lq = -iAlpha * sin(θ) + iBeta * cos(θ)
*/
void Park_Cala(p_SVGENDQ s)
{
    s->Id = _IQmpy(s->Ialpha, s->Cosine) + _IQmpy(s->Ibeta, s->Sine);
    s->Iq = _IQmpy(s->Ibeta, s->Cosine) - _IQmpy(s->Ialpha, s->Sine);
}
/*
InvPark：
    Valpha = vd * cos(θ) - vq * sin(θ)
    Vbeta  = vd * sin(θ) + vq * cos(θ)
*/
void InvPark(p_SVGENDQ s)
{
    s->Valpha = _IQmpy(s->Vd, s->Cosine) - _IQmpy(s->Vq, s->Sine);
    s->Vbeta = _IQmpy(s->Vd, s->Sine) + _IQmpy(s->Vq, s->Cosine);
}
