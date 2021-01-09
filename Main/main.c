#include "PI.h"
#include "smc.h"
#include "pwm.h"
#include "init.h"
#include "IQmath.h"
#include "common.h"
#include "control.h"
#include "svgen_dq.h"

int main()
{
	Clk_init();
	GPIO_init();
	PWM_init();
	DMA_init();
	OPA_init();
	ADC_init();
	UART_init();
	PowerupParaInit();
	Common_Init(); // 变量初始化
	delay1ms(200);
	while (1)
	{
		MotorControl(); //电机控制
	}
}
