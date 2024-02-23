#include <Modules/TempExt.h>
#include <cstdio>
#include <cstring>
#include <STRHAL.h>

TempExt::TempExt(
		const STRHAL_GPIO_t &addrPin0,
		const STRHAL_GPIO_t &addrPin1,
		const STRHAL_GPIO_t &addrPin2,
		const STRHAL_GPIO_t &addrPin3,
		const STRHAL_GPIO_Value_t &addr0,
		const STRHAL_GPIO_Value_t &addr1,
		const STRHAL_GPIO_Value_t &addr2,
		const STRHAL_GPIO_Value_t &addr3,
		const STRHAL_GPIO_t &muxEnable,
		const STRHAL_GPIO_t &spiData
		): addrPin0(addrPin0), addrPin1(addrPin1), addrPin2(addrPin2), addrPin3(addrPin3), addr0(addr0), addr1(addr1), addr2(addr2), addr3(addr3), muxEnable(muxEnable), spiData(spiData)
{
}

int TempExt::init()
{
	STRHAL_UART_Debug_Write_Blocking("TEMP EXT INIT\n", 14, 50);

	STRHAL_GPIO_SingleInit(&addrPin0, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&addrPin1, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&addrPin2, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&addrPin3, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&muxEnable, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&spiData, STRHAL_GPIO_TYPE_OPP);

	STRHAL_GPIO_Write(&addrPin0, addr0);
	STRHAL_GPIO_Write(&addrPin1, addr1);
	STRHAL_GPIO_Write(&addrPin2, addr2);
	STRHAL_GPIO_Write(&addrPin3, addr3);
	STRHAL_GPIO_Write(&muxEnable, STRHAL_GPIO_VALUE_L);
	STRHAL_GPIO_Write(&spiData, STRHAL_GPIO_VALUE_H);

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
