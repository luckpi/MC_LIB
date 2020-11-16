#include "IQmath.h"
#include "atan2.h"
/* atan函数 */
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
    zz = (z * z) >> 15;
    temp = (RL_A * zz) >> 15;
    temp = (temp + RL_B) * zz >> 15;
    temp = (temp + RL_C) * z >> 15;
    return (temp);
}
/* atan2函数 */
int16_t Atan2(int16_t y, int16_t x)
{
    int16_t tmp1, tmp2, z;

    tmp1 = Abs(y);
    tmp2 = Abs(x);
    z = (tmp1 > tmp2) ? (tmp1) : (tmp2); /* Pick the max value from tmp1 and tmp2. */

    if (z < 1)
    {
        return 0;
    }
    if (tmp2 >= tmp1)
    {
        z = ((y << 15) / x);
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
        z = ((int32_t)(x << 15) / y);
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
