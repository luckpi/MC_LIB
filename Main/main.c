#include "init.h"
#include "pwm.h"
#include "common.h"
#include "control.h"
#include "svgen_dq.h"
#include "IQmath.h"
#include "smc.h"
int main()
{
	Clk_Init();
	LED_Init();
	PWM_Init();
	DMA_Init();
	OPA_GpMode_Test();
	ADC_Init();
	UART_Init();
	PowerupParaInit();
	delay1ms(200);
	while (1)
	{

		MotorControl(); //电机控制

		// // printf("%d\t", Halless.HallessState);
		// switch (mcState)
		// {
		// case mcRun:
		// 	// Protect_Voltage();
		// 	// Protect_Current();
		// 	break;
		// default:
		// 	break;
		// }
		// printf("voltage=%d,POT=%d\n", ADCSample.Voltage, ADCSample.POT);
		// delay1ms(1);
		printf("Ia=%d,IaH=%d,Ualpha=%d\n", smc.Ialpha, smc.EstIalpha, smc.Valpha);
		// printf("Ia=%d,Ib=%d\n", smc.Ialpha, smc.Ibeta);
		// printf("Ealpha=%d,Ebeta=%d\n", smc.Ealpha, smc.Ebeta);
		// printf("ETheta=%d,Theta=%d\n", smc.Theta, AngleSin_Cos.IQAngle);
	}
}
