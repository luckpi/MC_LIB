#include "IQmath.h"
#include "atan2.h"
/* description: Polynomial approximated atan function with float parameters and return value. */
static int16_t polynmApproxAtanf(int16_t tanVal)
{
    int16_t z, zz, temp;

    /* z should be inside [-1.0, 1.0]. */
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
/* description: Polynomial approximated atan2 function with float parameters and return value. */
int16_t polynmApproxAtan2f(int16_t y, int16_t x)
{
    int16_t tmp1, tmp2, z;

    tmp1 = Abs(y);
    tmp2 = Abs(x);
    z = (tmp1 > tmp2) ? (tmp1) : (tmp2); /* Pick the max value from tmp1 and tmp2. */

    if (z < 1)
    { /* Both y and x are too small. */
        return 0;
    }

    if (tmp2 >= tmp1)
    { /* abs(x) >= abs(y) */
        z = ((y << 15) / x);
        tmp2 = polynmApproxAtanf(z);
        if (x > 0)
        { /* -pi/2 ~ pi/2 */
            tmp1 = tmp2;
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
    { /* abs(x) < abs(y) */
        z = ((int32_t)(x << 15) / y);
        tmp2 = polynmApproxAtanf(z);
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
