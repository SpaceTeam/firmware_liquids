#ifndef __SX1276_H__
#define __SX1276_H__

#include <inttypes.h>
#include "SX1276_regs.h"
#include <STRHAL.h>
#include <cstdio>

#include <array>
#include <vector>
#include <string.h>
#include <mutex>
#include <climits>

#define SX1276_OK 0
#define SX1276_ERROR 1

typedef struct
{
	uint32_t frequency;
	uint8_t txPower;
	uint8_t spreadingFactor;
	uint32_t signalBandwith;
	uint8_t codingRateDenominator;
	uint16_t preambleLength;
	uint8_t syncword;
	uint8_t crc;
	uint8_t messageSize;
} loraSettings_t;

typedef enum
{
	idle,
	tx,
	rx,
	sleep,
	lora_sleep,
	disconnected
} loraStatus_e;

class SX1276
{
protected:
	SX1276 *instance;

public:
	SX1276(const STRHAL_SPI_Id_t &spiId, const STRHAL_SPI_Config_t &spiConf, const STRHAL_GPIO_t &dio0, const STRHAL_GPIO_t &reset);
	virtual ~SX1276() {};

	unsigned char init(const loraSettings_t *settings);
	loraStatus_e getStatus();
	uint8_t ready();
	int sendBytes(const uint8_t *buffer, uint8_t length);
	void setMessageSize(uint8_t size);
	uint8_t getMessageSize();
	void setReceive();
	uint8_t parsePacket();
	uint8_t packetRssi();
	float packetSnr();
	uint8_t available();
	int16_t read();
	uint16_t readBytes(uint8_t *buffer, uint16_t length);
	int16_t peek();
	int8_t getTemperature();
	void setLoraMode();
	void setIdle();
	void setSleep();
	void setTxPower(uint8_t level);
	void setFrequency(uint32_t frequency);
	void setSpreadingFactor(uint8_t sf);
	void setSignalBandwidth(uint32_t sbw);
	void setCodingRate4(uint8_t denominator);
	void setPreambleLength(uint16_t length);
	void setSyncWord(uint8_t sw);
	uint8_t getMode();
	void crc();
	void noCrc();
	uint8_t random();
	void explicitHeaderMode();
	void implicitHeaderMode();

	bool messageReceived();

	uint8_t Send(uint8_t* buffer, size_t len);
	template <const size_t dataToSend>
	uint8_t SendReceive(std::array<uint8_t, dataToSend> &data, std::array<uint8_t, dataToSend> &recv, const size_t toSend = dataToSend);

private:
	uint8_t singleTransfer(uint8_t address, uint8_t value);
	void fifoTransfer(uint8_t address, const uint8_t* buffer, size_t len);
	uint8_t readRegister(uint8_t address);
	void writeRegister(uint8_t address, uint8_t value);
	void writeRegisterSafe(uint8_t address, uint8_t value);
	void Reset(bool reconfigure);
	void ConfigureLora();

	loraSettings_t currentSettings;
	uint32_t _frequency;
	int _packetIndex;
	uint8_t messageSize;

	STRHAL_SPI_Id_t spiId;
	STRHAL_SPI_Config_t spiConf;
	const STRHAL_GPIO_t dio0;
	const STRHAL_GPIO_t reset;

	volatile uint8_t loraLocked;

	volatile bool messagePending = false;
};

#endif
