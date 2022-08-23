#include "stm32g4xx_it.h"
#include <STRHAL.h>

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
	uint8_t alt = 0;
	while (1)
	{
		if (alt)
			LL_GPIO_SetOutputPin(GPIOD, (1 << 1));
		else
			LL_GPIO_ResetOutputPin(GPIOD, (1 << 1));

		alt = !alt;

	}
}

void MemManage_Handler(void)
{
	while (1)
	{
	}
}

void BusFault_Handler(void)
{
	while (1)
	{
	}
}

void UsageFault_Handler(void)
{
	while (1)
	{
	}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

//void SysTick_Handler(void)
//{
//}
