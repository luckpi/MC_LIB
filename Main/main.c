#include "PI.h"
#include "smc.h"
#include "pwm.h"
#include "Hall.h"
#include "init.h"
#include "IQmath.h"
#include "common.h"
#include "control.h"
#include "svgen_dq.h"

int main()
{
	Clk_init();
	LED_init();
	PWM_init();
	DMA_init();
	OPA_init();
	ADC_init();
	UART_init();
	Hall_init();
	PowerupParaInit();
	Common_Init(); // 变量初始化
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
		// printf("Ia=%d,IaH=%d,Ualpha=%d\n", smc.Ialpha, smc.EstIalpha, smc.Valpha);
		// printf("IaH=%d,Ualpha=%d\n", smc.EstIalpha, smc.Valpha);
		// printf("Ia=%d,IaH=%d\n", smc.Ialpha, smc.EstIalpha);
		// printf("Ib=%d,IbH=%d\n", smc.Ibeta, smc.EstIbeta);
		// printf("Ualpha=%d,Ubeta=%d\n", smc.Valpha, smc.Vbeta);
		// printf("Ia=%d,Ib=%d\n", SVM.Ia, SVM.Ib);
		// printf("Vq=%d,Vd=%d\n", SVM.Vq, SVM.Vd);
		printf("Iq=%d,Id=%d\n", SVM.Iq, SVM.Id);
		// printf("Og=%d\n", smc.Omega);
		// printf("Ealpha=%d,Ebeta=%d\n", smc.Ealpha, smc.Ebeta);
		// printf("ETheta=%d,Theta=%d\n", smc.Theta, AngleSin_Cos.IQAngle);
		// printf("Hall=%d\n", AngleSin_Cos.Angle_X);
	}
}
