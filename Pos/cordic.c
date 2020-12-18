#include "smc.h"

#define ATAN1DIV1       (int16_t)8192
#define ATAN1DIV2       (int16_t)4836
#define ATAN1DIV4       (int16_t)2555
#define ATAN1DIV8       (int16_t)1297
#define ATAN1DIV16      (int16_t)651
#define ATAN1DIV32      (int16_t)326
#define ATAN1DIV64      (int16_t)163
#define ATAN1DIV128     (int16_t)81
#define ATAN1DIV256     (int16_t)41
#define ATAN1DIV512     (int16_t)20
#define ATAN1DIV1024    (int16_t)10
#define ATAN1DIV2048    (int16_t)5
#define ATAN1DIV4096    (int16_t)3
#define ATAN1DIV8192    (int16_t)1

// uint16_t atandiv[] = {8192, 4836, 2555, 1297,
//                       651, 326, 163, 81, 41,
//                       20, 10, 5, 3, 1};
int16_t CORDIC_Atan(int16_t x, int16_t y)
{
    int16_t hAngle;
    int32_t wXi, wYi, wXold;
    /* 确定象限 */
    if (x < 0)
    {
        if (y < 0)
        {
            /* 象限III，相加90度即可移至象限IV */
            hAngle = 16384;
            wXi = -(y >> 1);
            wYi = x >> 1;
        }
        else
        {
            /* 象限II，减去90度即可移至象限I */
            hAngle = -16384;
            wXi = y >> 1;
            wYi = -(x >> 1);
        }
    }
    else
    {
        /* 象限I或IV */
        hAngle = 0;
        wXi = x >> 1;
        wYi = y >> 1;
    }
    wXold = wXi;
    // for (uint8_t i = 0; i < 8; i++)
    // {
    //     wXold = wXi;
    //     /* 开始逐次逼近过程 */
    //     if (wYi < 0)
    //     {
    //         /* 向量在IV象限中 */
    //         hAngle += atandiv[i];
    //         wXi -= wYi >> i;
    //         wYi += wXold >> i;
    //     }
    //     else
    //     {
    //         /* 向量在象限I中 */
    //         hAngle -= atandiv[i];
    //         wXi += wYi >> i;
    //         wYi -= wXold >> i;
    //     }
    // }
    /*begin the successive approximation process*/
    /*iteration0*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV1;
        wXi = wXi - wYi;
        wYi = wXold + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV1;
        wXi = wXi + wYi;
        wYi = -wXold + wYi;
    }
    wXold = wXi;

    /*iteration1*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV2;
        wXi -= wYi >> 1;
        wYi += wXold >> 1;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV2;
        wXi += wYi >> 1;
        wYi -= wXold >> 1;
    }
    wXold = wXi;

    /*iteration2*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV4;
        wXi -= wYi >> 2;
        wYi += wXold >> 2;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV4;
        wXi += wYi >> 2;
        wYi -= wXold >> 2;
    }
    wXold = wXi;

    /*iteration3*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV8;
        wXi -= wYi >> 3;
        wYi += wXold >> 3;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV8;
        wXi += wYi >> 3;
        wYi -= wXold >> 3;
    }
    wXold = wXi;

    /*iteration4*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV16;
        wXi -= wYi >> 4;
        wYi += wXold >> 4;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV16;
        wXi += wYi >> 4;
        wYi -= wXold >> 4;
    }
    wXold = wXi;

    /*iteration5*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV32;
        wXi -= wYi >> 5;
        wYi += wXold >> 5;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV32;
        wXi += wYi >> 5;
        wYi -= wXold >> 5;
    }
    wXold = wXi;

    /*iteration6*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV64;
        wXi -= wYi >> 6;
        wYi += wXold >> 6;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV64;
        wXi += wYi >> 6;
        wYi -= wXold >> 6;
    }
    wXold = wXi;

    /*iteration7*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += ATAN1DIV128;
        wXi -= wYi >> 7;
        wYi += wXold >> 7;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= ATAN1DIV128;
        wXi += wYi >> 7;
        wYi -= wXold >> 7;
    }
    return (-hAngle);
}
