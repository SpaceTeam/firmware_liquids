#ifndef TEMP_EXT_H
#define TEMP_EXT_H

#include <STRHAL.h>
#include "./Modules/AbstractModule.h"

enum class TempExtAddr : uint8_t
{
	DATA_LSB = 0x02,
	DATA_MSB = 0x01
};


class TempExt: public AbstractModule
{
	public:
		TempExt(
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

		static constexpr uint64_t EXEC_SAMPLE_TICKS = 10;

	private:
		bool readReg(const TempExtAddr &address, uint8_t *reg);

		uint8_t deviceID;
		STRHAL_GPIO_t addrPin0, addrPin1, addrPin2, addrPin3, muxEnable;
		STRHAL_GPIO_Value_t addr0, addr1, addr2, addr3;
		STRHAL_SPI_Id_t spiId;
		STRHAL_SPI_Config_t spiConf;

		uint64_t timeLastSample = 0;
};

#endif /*TEMP_EXT_H*/
