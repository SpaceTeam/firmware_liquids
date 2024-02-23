#ifndef TEMP_EXT_H
#define TEMP_EXT_H

#include <STRHAL.h>
#include "./Modules/AbstractModule.h"


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
				const STRHAL_GPIO_t &spiData
			);

		int init() override;
		int exec() override;
		int reset() override;

	private:
		STRHAL_SPI_Id_t spiId;
		STRHAL_SPI_Config_t spiConf;
		uint8_t deviceID;
		STRHAL_GPIO_t addrPin0, addrPin1, addrPin2, addrPin3, muxEnable, spiData;
		STRHAL_GPIO_Value_t addr0, addr1, addr2, addr3;
};

#endif /*TEMP_EXT_H*/
