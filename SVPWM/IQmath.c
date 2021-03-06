#include "common.h"
#include "IQmath.h"

// SINCOS 0-90°表
const int16_t IQSin_Cos_Table[256] = {
    0x0000, 0x00C9, 0x0192, 0x025B, 0x0324, 0x03ED, 0x04B6, 0x057F,
    0x0648, 0x0711, 0x07D9, 0x08A2, 0x096A, 0x0A33, 0x0AFB, 0x0BC4,
    0x0C8C, 0x0D54, 0x0E1C, 0x0EE3, 0x0FAB, 0x1072, 0x113A, 0x1201,
    0x12C8, 0x138F, 0x1455, 0x151C, 0x15E2, 0x16A8, 0x176E, 0x1833,
    0x18F9, 0x19BE, 0x1A82, 0x1B47, 0x1C0B, 0x1CCF, 0x1D93, 0x1E57,
    0x1F1A, 0x1FDD, 0x209F, 0x2161, 0x2223, 0x22E5, 0x23A6, 0x2467,
    0x2528, 0x25E8, 0x26A8, 0x2767, 0x2826, 0x28E5, 0x29A3, 0x2A61,
    0x2B1F, 0x2BDC, 0x2C99, 0x2D55, 0x2E11, 0x2ECC, 0x2F87, 0x3041,
    0x30FB, 0x31B5, 0x326E, 0x3326, 0x33DF, 0x3496, 0x354D, 0x3604,
    0x36BA, 0x376F, 0x3824, 0x38D9, 0x398C, 0x3A40, 0x3AF2, 0x3BA5,
    0x3C56, 0x3D07, 0x3DB8, 0x3E68, 0x3F17, 0x3FC5, 0x4073, 0x4121,
    0x41CE, 0x427A, 0x4325, 0x43D0, 0x447A, 0x4524, 0x45CD, 0x4675,
    0x471C, 0x47C3, 0x4869, 0x490F, 0x49B4, 0x4A58, 0x4AFB, 0x4B9D,
    0x4C3F, 0x4CE0, 0x4D81, 0x4E20, 0x4EBF, 0x4F5D, 0x4FFB, 0x5097,
    0x5133, 0x51CE, 0x5268, 0x5302, 0x539B, 0x5432, 0x54C9, 0x5560,
    0x55F5, 0x568A, 0x571D, 0x57B0, 0x5842, 0x58D3, 0x5964, 0x59F3,
    0x5A82, 0x5B0F, 0x5B9C, 0x5C28, 0x5CB3, 0x5D3E, 0x5DC7, 0x5E4F,
    0x5ED7, 0x5F5D, 0x5FE3, 0x6068, 0x60EB, 0x616E, 0x61F0, 0x6271,
    0x62F1, 0x6370, 0x63EE, 0x646C, 0x64E8, 0x6563, 0x65DD, 0x6656,
    0x66CF, 0x6746, 0x67BC, 0x6832, 0x68A6, 0x6919, 0x698B, 0x69FD,
    0x6A6D, 0x6ADC, 0x6B4A, 0x6BB7, 0x6C23, 0x6C8E, 0x6CF8, 0x6D61,
    0x6DC9, 0x6E30, 0x6E96, 0x6EFB, 0x6F5E, 0x6FC1, 0x7022, 0x7083,
    0x70E2, 0x7140, 0x719D, 0x71F9, 0x7254, 0x72AE, 0x7307, 0x735E,
    0x73B5, 0x740A, 0x745F, 0x74B2, 0x7504, 0x7555, 0x75A5, 0x75F3,
    0x7641, 0x768D, 0x76D8, 0x7722, 0x776B, 0x77B3, 0x77FA, 0x783F,
    0x7884, 0x78C7, 0x7909, 0x794A, 0x7989, 0x79C8, 0x7A05, 0x7A41,
    0x7A7C, 0x7AB6, 0x7AEE, 0x7B26, 0x7B5C, 0x7B91, 0x7BC5, 0x7BF8,
    0x7C29, 0x7C59, 0x7C88, 0x7CB6, 0x7CE3, 0x7D0E, 0x7D39, 0x7D62,
    0x7D89, 0x7DB0, 0x7DD5, 0x7DFA, 0x7E1D, 0x7E3E, 0x7E5F, 0x7E7E,
    0x7E9C, 0x7EB9, 0x7ED5, 0x7EEF, 0x7F09, 0x7F21, 0x7F37, 0x7F4D,
    0x7F61, 0x7F74, 0x7F86, 0x7F97, 0x7FA6, 0x7FB4, 0x7FC1, 0x7FCD,
    0x7FD8, 0x7FE1, 0x7FE9, 0x7FF0, 0x7FF5, 0x7FF9, 0x7FFD, 0x7FFE};

/*****************************************************************************
 函 数 名  : IQSin_Cos_Cale
 功能描述  : SIN COS查表法计算
 输入参数  : 角度
 输出参数  : void
*****************************************************************************/
void IQSin_Cos_Cale(p_SVGENDQ pV)
{
    uint16_t hindex;
    hindex = (uint16_t)pV->Theta; // -32768--32767 +32768 = 0--65535
    hindex >>= 6;                 // 65536 / 64 = 1024 / 4 = 90° = 256  0x01-0xFF  0X100  0X1FF  0X200 0X2FF    0X300  0X3FF

    switch (hindex & SIN_RAD) //  0X300  &  0x0XXX  获取象限
    {
    case U0_90:
        pV->Sine = IQSin_Cos_Table[(uint8_t)(hindex)]; // 0---255  ==0---32766
        pV->Cosine = IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))];
        break;

    case U90_180:
        pV->Sine = IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))]; // 255---0 == 0---32766
        pV->Cosine = -IQSin_Cos_Table[(uint8_t)(hindex)];
        break;

    case U180_270:
        pV->Sine = -IQSin_Cos_Table[(uint8_t)(hindex)];
        pV->Cosine = -IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))];
        break;

    case U270_360:
        pV->Sine = -IQSin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(hindex))];
        pV->Cosine = IQSin_Cos_Table[(uint8_t)(hindex)];
        break;
    default:
        break;
    }
}
/*****************************************************************************
 函 数 名  : IQsat
 功能描述  : 输出限幅
 输入参数  : 比较值，最大值，最小值
 输出参数  : void
*****************************************************************************/
int32_t IQsat(int32_t Uint, int32_t U_max, int32_t U_min)
{
    int32_t Uout;
    if (Uint <= U_min)
        Uout = U_min;
    else if (Uint >= U_max)
        Uout = U_max;
    else
        Uout = Uint;

    return Uout;
}
/*****************************************************************************
 函 数 名  : IQSqrt
 功能描述  : 0 ~ 2^32开方
 输入参数  : 被开方数
 输出参数  : void
*****************************************************************************/
uint32_t IQSqrt(uint32_t M)
{
    uint32_t Input = M;
    uint32_t SqrtX = 0;
    uint32_t Temp = 0;
    if (M <= 0)
        return 0;
    Temp = SqrtX + (1 << 30);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 30);
    }
    Temp = SqrtX + (1 << 28);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 28);
    }
    Temp = SqrtX + (1 << 26);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 26);
    }
    Temp = SqrtX + (1 << 24);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 24);
    }
    Temp = SqrtX + (1 << 22);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 22);
    }
    Temp = SqrtX + (1 << 20);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 20);
    }
    Temp = SqrtX + (1 << 18);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 18);
    }
    Temp = SqrtX + (1 << 16);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 16);
    }
    Temp = SqrtX + (1 << 14);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 14);
    }
    Temp = SqrtX + (1 << 12);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 12);
    }
    Temp = SqrtX + (1 << 10);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 10);
    }
    Temp = SqrtX + (1 << 8);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 8);
    }
    Temp = SqrtX + (1 << 6);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 6);
    }
    Temp = SqrtX + (1 << 4);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 4);
    }
    Temp = SqrtX + (1 << 2);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 2);
    }
    Temp = SqrtX + (1 << 0);
    SqrtX >>= 1;
    if (Temp <= Input)
    {
        Input -= Temp;
        SqrtX += (1 << 0);
    }
    return ((int32_t)(SqrtX));
}
/*****************************************************************************
 函 数 名  : HDIV
 功能描述  : 硬件除法器，需硬件支持
 输入参数  : 被除数,  除数
 输出参数  : void
*****************************************************************************/
int32_t HDIV(int32_t Dividend, int16_t Divisor)
{
    M0P_HDIV->SIGN_f.SIGN = 1;           // 有无符号除法 1：有符号 0：无符号
    (M0P_HDIV->DIVIDEND) = Dividend;     // 被除数
    (M0P_HDIV->DIVISOR) = Divisor;       // 除数
    while (M0P_HDIV->STAT_f.END != TRUE) // 等待除法运算结束
        ;
    //M0P_HDIV->QUOTIENT_f.QUOTIENT;   //商
    //M0P_HDIV->REMAINDER_f.REMAINDER; //余数
    return (M0P_HDIV->QUOTIENT_f.QUOTIENT);
}
