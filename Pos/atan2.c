#include "atan2.h"
#include "IQmath.h"
/*****************************************************************************
 函 数 名  : Atan
 功能描述  : 反正切
 输入参数  : α，β轴的商
 输出参数  : void
*****************************************************************************/
static int16_t Atan(int16_t tanVal)
{
    int16_t z, zz, temp;

    /* Z的值域 [-1.0, 1.0] */
    if (32767 < tanVal)
    {
        z = 32767;
    }
    else if (-32768 > tanVal)
    {
        z = -32768;
    }
    else
    {
        z = tanVal;
    }
    zz = (int16_t)(_IQmpy(z, z));
    temp = (int16_t)(_IQmpy(RL_A, zz));
    temp = (int16_t)(_IQmpy((temp + RL_B), zz));
    temp = (int16_t)(_IQmpy((temp + RL_C), z));
    return (temp);
}
/*****************************************************************************
 函 数 名  : Atan2
 功能描述  : 反正切
 输入参数  : α，β轴估算反电动势
 输出参数  : 角度
*****************************************************************************/
int16_t Atan2(int16_t y, int16_t x)
{
    int16_t tmp1, tmp2, z;

    tmp1 = Abs(y);
    tmp2 = Abs(x);
    z = (tmp1 > tmp2) ? (tmp1) : (tmp2); // 比较最大值

    if (z < RL_EPSILON_F)
    {
        return 0;
    }
    if (tmp2 >= tmp1)
    {
        z = HDIV_div((y << 15), x);
        tmp2 = Atan(z);
        if (x > 0)
        {
            tmp1 = tmp2; // -180° ~ 180°
        }
        else if (y > 0)
        {
            tmp1 = tmp2 + M_PI;
        }
        else
        {
            tmp1 = tmp2 - M_PI;
        }
    }
    else
    {
        z = HDIV_div((x << 15), y);
        tmp2 = Atan(z);
        if (y > 0)
        {
            tmp1 = -tmp2 + M_PI_2;
        }
        else
        {
            tmp1 = -tmp2 - M_PI_2;
        }
    }

    return tmp1;
}
