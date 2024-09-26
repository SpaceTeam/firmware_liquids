#include <Modules/MAX31865_Temp.h>
#include <cstdio>
#include <cstring>
#include <STRHAL.h>
#include <iostream>

MAX31865_Temp::MAX31865_Temp(
		const STRHAL_SPI_Id_t &spiId,
		const STRHAL_SPI_Config_t &spiConf
		): 	spiId(spiId),
			spiConf(spiConf)
{
}

int MAX31865_Temp::init()
{
	STRHAL_UART_Debug_Write_Blocking("TEMP EXT INIT\n", 14, 50);

	STRHAL_SPI_Init();
	if (STRHAL_SPI_Master_Init(spiId, &spiConf) < 0)
		return -1;

	STRHAL_SPI_Master_Run(spiId);

	writeReg(MAX31865_Temp_Addr::CONFIG, 0xC1, 50);

	return 0;
}

int MAX31865_Temp::exec()
{
	uint64_t time = STRHAL_Systick_GetTick();

	if ((time - timeLastSample) > EXEC_SAMPLE_TICKS){
		timeLastSample = time;
		measurementData = readData();
		new_data = 1;
	}
	return 0;
}

STRHAL_SPI_NSSId_t MAX31865_Temp::Get_NSS(void)
{
	return spiConf.nss;
}

uint8_t MAX31865_Temp::readSingleReg(const MAX31865_Temp_Addr &address)
{
	STRHAL_SPI_Select_Chip(spiId, &spiConf);
	uint8_t cmd = static_cast<uint8_t>(address);
	uint32_t value;
	if (STRHAL_SPI_Master_Transceive(spiId, &cmd, 1, 0, (uint8_t*)&value, 2, 500) < 0){
		return -1;
	}
	return (uint8_t)(value >> 8);
}
uint16_t MAX31865_Temp::readData(void)
{
	STRHAL_SPI_Select_Chip(spiId, &spiConf);
	uint8_t cmd = static_cast<uint8_t>(MAX31865_Temp_Addr::DATA_MSB);
	uint32_t value;

	if (STRHAL_SPI_Master_Transceive(spiId, &cmd, 1, 0, (uint8_t*)&value, 4, 500) < 0){
		return -1;
	}
	return (uint16_t)(((value >> 16) & 0x00ff) | ((value >> 0) & 0xff00));
}

uint8_t MAX31865_Temp::writeReg(const MAX31865_Temp_Addr &address, uint8_t val, uint16_t delay)//UNTESTED
{

	STRHAL_SPI_Select_Chip(spiId, &spiConf);
	uint8_t cmd[2];

	cmd[0] = static_cast<uint8_t>(address);
	cmd[1] = val;

	if (STRHAL_SPI_Master_Transceive(spiId, cmd, 2, 2, nullptr, 0, 100) != 0)
		return -1;

	LL_mDelay(delay);
	return 0;
}

int MAX31865_Temp::reset()
{
	//STRHAL_UART_Debug_Write_Blocking("TEMP EXT RESET\n", 15, 50);
	return 0;
}
