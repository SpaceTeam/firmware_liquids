#ifndef MAX31865_H
#define MAX31865_H

#include <STRHAL.h>
#include "./Modules/AbstractModule.h"

enum class MAX31865_Temp_Addr : uint8_t
{
	DATA_MSB = 0x01,
	DATA_LSB = 0x02,
	DATA_HIGH_FAULT_MSB = 0x03,
	DATA_HIGH_FAULT_LSB = 0x04,
	DATA_LOW_FAULT_MSB = 0x05,
	DATA_LOW_FAULT_LSB = 0x06,
	FAULT_STATUS = 0x07,
	CONFIG = 0x80
};


class MAX31865_Temp: public AbstractModule
{
	public:
		MAX31865_Temp(
				const STRHAL_SPI_Id_t &spiId,
				const STRHAL_SPI_Config_t &spiConf
			);

		int init() override;
		int exec() override;
		int reset() override;

		static constexpr uint64_t EXEC_SAMPLE_TICKS = 100;

		uint8_t new_data = 0;
		uint16_t measurementData = 0;

		STRHAL_SPI_NSSId_t Get_NSS(void);
	private:
		uint8_t readSingleReg(const MAX31865_Temp_Addr &address);
		uint16_t readData(void);

		uint8_t writeReg(const MAX31865_Temp_Addr &address, uint8_t val, uint16_t delay);

		uint8_t deviceID;
		STRHAL_SPI_Id_t spiId;
		STRHAL_SPI_Config_t spiConf;
		uint64_t timeLastSample = 0;
};

#endif /*MAX1865_H*/
