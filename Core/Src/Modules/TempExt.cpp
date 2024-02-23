#include <Modules/TempExt.h>
#include <cstdio>
#include <cstring>
#include <STRHAL.h>

TempExt::TempExt(const STRHAL_GPIO_t &addrPin0, const STRHAL_GPIO_t &addrPin1, const STRHAL_GPIO_t &addrPin2, const STRHAL_GPIO_t &addrPin3):
	addrPin0(addrPin0), addrPin1(addrPin1), addrPin2(addrPin2), addrPin3(addrPin3)
{
}

int TempExt::init()
{
	STRHAL_UART_Debug_Write_Blocking("TEMP EXT INIT\n", 14, 50);

	STRHAL_GPIO_SingleInit(&addrPin0, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&addrPin1, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&addrPin2, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&addrPin3, STRHAL_GPIO_TYPE_OPP);

	return 0;
}

int TempExt::exec()
{
	STRHAL_UART_Debug_Write_Blocking("TEMP EXT EXEC\n", 14, 50);
	return 0;
}

int TempExt::reset()
{
	STRHAL_UART_Debug_Write_Blocking("TEMP EXT RESET\n", 15, 50);
	return 0;
}
