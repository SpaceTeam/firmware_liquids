#ifndef TEMP_EXT_H
#define TEMP_EXT_H

#include <STRHAL.h>
#include "./Modules/AbstractModule.h"


class TempExt: public AbstractModule
{
	public:
		TempExt(const STRHAL_SPI_Id_t &spiId, const STRHAL_SPI_Config_t &spiConf, const STRHAL_GPIO_t &dataReadyPin, uint8_t deviceID);

		int init() override;
		int exec() override;
		int reset() override;

	private:
		STRHAL_SPI_Id_t spiId;
		STRHAL_SPI_Config_t spiConf;
		const STRHAL_GPIO_t dataReadyPin;
		uint8_t deviceID;

};

#endif /*TEMP_EXT_H*/
