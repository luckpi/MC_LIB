#include "smc.h"
uint16_t atandiv[] = {8192, 4836, 2555, 1297,
                      651, 326, 163, 81, 41,
                      20, 10, 5, 3, 1};
int16_t CORDIC_Atan(int16_t alfa_est, int16_t beta_est)
{
    int16_t hAngle;
    int32_t wXi, wYi, wXold;
    /* 确定象限 */
    if (alfa_est < 0)
    {
        if (beta_est < 0)
        {
            /* 象限III，相加90度即可移至象限IV */
            hAngle = 16384;
            wXi = -(beta_est >> 1);
            wYi = alfa_est >> 1;
        }
        else
        {
            /* 象限II，减去90度即可移至象限I */
            hAngle = -16384;
            wXi = beta_est >> 1;
            wYi = -(alfa_est >> 1);
        }
    }
    else
    {
        /* 象限I或IV */
        hAngle = 0;
        wXi = alfa_est >> 1;
        wYi = beta_est >> 1;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        wXold = wXi;
        /* 开始逐次逼近过程 */
        if (wYi < 0)
        {
            /* 向量在IV象限中 */
            hAngle += atandiv[i];
            wXi -= wYi >> i;
            wYi += wXold >> i;
        }
        else
        {
            /* 向量在象限I中 */
            hAngle -= atandiv[i];
            wXi += wYi >> i;
            wYi -= wXold >> i;
        }
    }
    return (hAngle);
}
