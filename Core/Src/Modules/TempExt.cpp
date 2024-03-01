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
		const STRHAL_SPI_Id_t &spiId,
		const STRHAL_SPI_Config_t &spiConf
		): 	addrPin0(addrPin0),
			addrPin1(addrPin1),
			addrPin2(addrPin2),
			addrPin3(addrPin3),
			addr0(addr0),
			addr1(addr1),
			addr2(addr2),
			addr3(addr3),
			muxEnable(muxEnable),
			spiId(spiId),
			spiConf(spiConf)
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

	STRHAL_GPIO_Write(&addrPin0, addr0);
	STRHAL_GPIO_Write(&addrPin1, addr1);
	STRHAL_GPIO_Write(&addrPin2, addr2);
	STRHAL_GPIO_Write(&addrPin3, addr3);
	STRHAL_GPIO_Write(&muxEnable, STRHAL_GPIO_VALUE_L);

	if (STRHAL_SPI_Master_Init(spiId, &spiConf) < 0)
		return -1;

	return 0;
}

int TempExt::exec()
{
	uint64_t time = STRHAL_Systick_GetTick();
	if ((time - timeLastSample) < EXEC_SAMPLE_TICKS)
		return 0;

	timeLastSample = time;
	(void) read();

	return 0;
}

int TempExt::read()
{
	uint8_t data;
	if(readReg(TempExtAddr::DATA_MSB, &data) < 0){
		STRHAL_UART_Debug_Write_Blocking("SPI ERROR\n", 9, 50);
		return -1;
	}
	const char data_char = (const char)data;
	STRHAL_UART_Debug_Write_Blocking("SPI READ\n", 9, 50);
	STRHAL_UART_Debug_Write_Blocking(&data_char, 9, 50);
}

bool TempExt::readReg(const TempExtAddr &address, uint8_t *reg)
{
	uint8_t cmd = static_cast<uint8_t>(address);

	return STRHAL_SPI_Master_Transceive(spiId, &cmd, 1, 0, reg, 1, 100);
}

int TempExt::reset()
{
	STRHAL_UART_Debug_Write_Blocking("TEMP EXT RESET\n", 15, 50);
	return 0;
}
