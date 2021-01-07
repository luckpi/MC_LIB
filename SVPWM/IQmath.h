#ifndef _IQMATH_H
#define _IQMATH_H
#include "common.h"
#include "svgen_dq.h"

#define Abs(A)              ((A >= 0) ? A : -A)
#define Min(A, B)           ((A <= B) ? A : B)
#define Max(A, B)           ((A >= B) ? A : B)
#define Q15(Float_Value)                                            \
        ((Float_Value < 0.0) ? (int32_t)(32768 * (Float_Value)-0.5) \
                             : (int32_t)(32767 * (Float_Value) + 0.5))
#define _IQmpy(A, B)        (int32_t)((A * B) >> 15)
#define _IQdiv2(A)          (int32_t)((A) >> 1)
#define _IQmpy2(A)          (int32_t)(A << 1)
#define _IQdiv(A, B)        (int32_t)((A << 15) / B)

#define SIN_RAD             0x0300
#define U0_90               0x0000
#define U90_180             0x0100
#define U180_270            0x0200
#define U270_360            0x0300


extern uint32_t IQSqrt(uint32_t M);
extern void IQSin_Cos_Cale(p_SVGENDQ pV);
extern int32_t IQsat(int32_t Uint, int32_t U_max, int32_t U_min);
extern int32_t HDIV(int32_t Dividend, int16_t Divisor);

#endif
