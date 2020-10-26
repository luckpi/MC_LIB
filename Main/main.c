#include "init.h"
#include "pwm.h"
#include "common.h"
#include "control.h"
int main()
{
	Clk_Init();
	LED_Init();
	PWM_Init();
	ADC_Init();
	UART_Init();
	Common_Init();
	PowerupParaInit();
	delay1ms(200);
	while (1)
	{
		MotorControl(); //电机控制
		// printf("%d\t", Halless.HallessState);
		switch (mcState)
		{
		case mcRun:
			// if (ADCSample.Voltage)
			// Protect_Voltage();
			// Ps("BAT=", ADCSample.Voltage);
			// Ps("Current = ", ADCSample.Average);
			// Protect_Current();
			break;
		default:
			break;
		}
		// printf("Ubemf=%d, Vbemf=%d, Wbemf=%d\r\n", ADCSample.UBemf, ADCSample.VBemf, ADCSample.WBemf);
	}
}
