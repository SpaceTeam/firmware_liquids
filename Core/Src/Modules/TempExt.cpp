#include <Modules/TempExt.h>
#include <cstdio>
#include <cstring>
#include <STRHAL.h>

TempExt::TempExt(const STRHAL_SPI_Id_t &spiId, const STRHAL_SPI_Config_t &spiConf, const STRHAL_GPIO_t &dataReadyPin, uint8_t deviceID) :
		spiId(spiId), spiConf(spiConf), dataReadyPin(dataReadyPin), deviceID(deviceID)
{
}

int TempExt::init()
{
	STRHAL_UART_Debug_Write_Blocking("TEMP EXT INIT\n", 14, 50);
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
