#ifndef MAX31865_H
#define MAX31865_H

#include <STRHAL.h>
#include "./Modules/AbstractModule.h"

enum class MAX31865_Temp_Addr : uint8_t
{
	DATA_LSB = 0x02,
	DATA_MSB = 0x01,
	CONFIG = 0x80
};


class MAX31865_Temp: public AbstractModule
{
	public:
		MAX31865_Temp(
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
			);

		int init() override;
		int exec() override;
		int reset() override;
		int read();

		static constexpr uint64_t EXEC_SAMPLE_TICKS = 1000;

	private:
		int readReg(const MAX31865_Temp_Addr &address, uint8_t *reg, uint8_t shift);
		int writeReg(const MAX31865_Temp_Addr &address, uint8_t val, uint16_t delay);

		uint8_t deviceID;
		STRHAL_GPIO_t addrPin0, addrPin1, addrPin2, addrPin3;
		STRHAL_GPIO_Value_t addr0, addr1, addr2, addr3;
		STRHAL_GPIO_t muxEnable;
		STRHAL_SPI_Id_t spiId;
		STRHAL_SPI_Config_t spiConf;

		uint64_t timeLastSample = 0;
};

#endif /*MAX1865_H*/
