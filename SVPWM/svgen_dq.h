#ifndef _SVPWM_H
#define _SVPWM_H
#include "common.h"
typedef struct
{
    // Clack和Park变量
    int16_t Ia;     // 直接测量
    int16_t Ib;     // 直接测量
    int16_t Ic;     // 根据Ia + Ib + Ic =0可得
    int32_t Ia_C;   // Ia校准误差
    int32_t Ib_C;   // Ia校准误差
    int16_t Ialpha; // Clark变换输出
    int16_t Ibeta;  // Clark变换输出
    int16_t Sine;   // 正弦
    int16_t Cosine; // 余弦
    int16_t Ld;     // Park d轴
    int16_t Lq;     // Park q轴
    int16_t Vd;     // PI输出
    int16_t Vq;     // PI输出
    int16_t Valpha; // Park逆变换输出
    int16_t Vbeta;  // Park逆变换输出
    int16_t Ta;     // SVP输出
    int16_t Tb;     // SVP输出
    int16_t Tc;     // SVP输出
} SVGENDQ, *p_SVGENDQ;
#define SVGENDQ_DEFAULTS                                     \
    {                                                        \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }
extern SVGENDQ SVM; // SVPWM
extern void svgendq_calc(void);
extern void Clark_Cala(void);
extern void Park_Cala(void);
extern void InvPark(void);
#endif
