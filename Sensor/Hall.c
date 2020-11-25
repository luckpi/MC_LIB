#include "Hall.h"
#include "IQmath.h"
#include "common.h"
uint16_t Hall_Get(void)
{
    uint8_t U, V, W, UVW;
    W = Gpio_GetInputIO(GpioPortC, GpioPin13);
    V = Gpio_GetInputIO(GpioPortD, GpioPin0);
    U = Gpio_GetInputIO(GpioPortD, GpioPin1);
    UVW = ((U << 2) | (V << 1) | W);
    AngleSin_Cos.Angle_X = HoldParm.SpeedLoopCnt;
    HoldParm.SpeedLoopCnt = 0;
    // if (UVW == 1)
    // {

    // }
    // else if (UVW == 3)
    // {
    //     Theat = 60;
    // }
    // else if (UVW == 2)
    // {
    //     Theat = 120;
    // }
    // else if (UVW == 6)
    // {
    //     Theat = 180;
    // }
    // else if (UVW == 4)
    // {
    //     Theat = 240;
    // }
    // else if (UVW == 5)
    // {
    //     Theat = 300;
    // }
    return UVW;
}
