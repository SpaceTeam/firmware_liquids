#include "Modules/SX1276.h"
#include "Modules/SX1276_regs.h"
#include <vector>
#include <chrono>
#include <thread>
#include <stm32g4xx_ll_exti.h>
#include <stm32g4xx_ll_system.h>

#include <cstring>
#include <cstdio>
#include <array>
#include <ctime>
#include <time.h>



// Interface functions
uint8_t SX1276::singleTransfer(uint8_t address, uint8_t value)
{
	const uint8_t data[] = {address, value};
	uint8_t recv;
	STRHAL_SPI_Master_Transceive(spiId, data, sizeof(data), 1, &recv, 1, 100);
	return recv;
}

void SX1276::fifoTransfer(uint8_t address, const uint8_t *buffer, size_t len)
{
	uint8_t dummy;
	std::vector<uint8_t> buf;
	buf.push_back(address | 0x80);
	for (size_t i = 0; i < len; i++)
	{
		buf.push_back(buffer[i]);
	}
	STRHAL_SPI_Master_Transceive(spiId, buf.data(), buf.size(), buf.size(), &dummy, 0, 100);
}

uint8_t SX1276::readRegister(uint8_t address)
{
	return singleTransfer(address & 0x7f, 0x00);
}

void SX1276::writeRegister(uint8_t address, uint8_t value)
{
	singleTransfer(address | 0x80, value);
}

void SX1276::writeRegisterSafe(uint8_t address, uint8_t value)
{
	uint8_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		setSleep();
	writeRegister(address, value);
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		writeRegister(REG_OP_MODE, mode);
}

// DIO0 Interrupt for send and receive


/**** Functions ****/
SX1276::SX1276(const STRHAL_SPI_Id_t &spiId, const STRHAL_SPI_Config_t &spiConf, const STRHAL_GPIO_t &dio0, const STRHAL_GPIO_t &reset) : spiId(spiId), spiConf(spiConf), dio0(dio0), reset(reset)
{

}

unsigned char SX1276::init(const loraSettings_t *settings)
{

	loraLocked = 0;
	STRHAL_GPIO_SingleInit(&dio0, STRHAL_GPIO_TYPE_IHZ);
	STRHAL_GPIO_SingleInit(&reset, STRHAL_GPIO_TYPE_IHZ);

	if (STRHAL_SPI_Master_Init(spiId, &spiConf) < 0)
		return -1;

	STRHAL_SPI_Master_Run(spiId);

	memcpy(&currentSettings, settings, sizeof(loraSettings_t));
	ConfigureLora();

	return 1;
}

void SX1276::ConfigureLora()
{
	setSleep();
	setFrequency(currentSettings.frequency);
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
	writeRegister(REG_OCP, 0x0B);
	writeRegister(REG_LNA, 0x23);
	writeRegister(0x36, 0x02); // See Errata note
	writeRegister(0x3A, 0x64); // See Errata note
	setTxPower(currentSettings.txPower);
	setSpreadingFactor(currentSettings.spreadingFactor);
	setCodingRate4(currentSettings.codingRateDenominator);
	setSignalBandwidth(currentSettings.signalBandwith);
	setPreambleLength(currentSettings.preambleLength);
	setSyncWord(currentSettings.syncword);
	setMessageSize(currentSettings.messageSize);
	if (currentSettings.crc)
	{
		crc();
	}
	else
	{
		noCrc();
	}

	setLoraMode();
}

void SX1276::Reset(bool reconfigure)
{
	setSleep();
	writeRegister(REG_DETECTION_OPTIMIZE, 0x83);
	writeRegister(0x2F, 0x40);
	if (reconfigure)
	{
		ConfigureLora();
	}
}

loraStatus_e SX1276::getStatus()
{
	uint8_t mode = getMode();
	if (mode == (MODE_LONG_RANGE_MODE | MODE_TX))
		return tx;
	if (mode == (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS) || mode == (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
		return rx;
	if (mode == (MODE_LONG_RANGE_MODE | MODE_STDBY))
		return idle;
	if (mode == (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		return tx;
	return disconnected;
}

uint8_t SX1276::ready()
{
	loraStatus_e stat = getStatus();
	return (stat != tx) && (stat != rx) && (stat != disconnected);
}

int SX1276::sendBytes( uint8_t *buffer, uint8_t length)
{
	int ret = length;
	loraStatus_e stat = getStatus();
	if ((stat == tx) || (stat == disconnected))
		return 0;
	setIdle();
	writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); // Clear irq
	writeRegister(LR_RegHopPeriod, 0x00);			// No FHSS
	writeRegister(REG_DIO_MAPPING_1, 1 << 6);
	writeRegister(REG_FIFO_ADDR_PTR, 0);
	fifoTransfer(REG_FIFO, buffer, length);
	if (messageSize == 0)
		writeRegister(REG_PAYLOAD_LENGTH, length);
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	auto startTime = std::chrono::system_clock::now();
	while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
	{
		if (std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::system_clock::now() - startTime)).count() > 200)
		{
			ret = -1;
			break;
		}
	}
	writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	stat = idle;
	return ret;
}

void SX1276::setMessageSize(uint8_t size)
{
	if (size > 0)
	{
		implicitHeaderMode();
		writeRegister(REG_PAYLOAD_LENGTH, size);
	}
	else
	{
		explicitHeaderMode();
	}
	messageSize = size;
}

uint8_t SX1276::getMessageSize()
{
	return messageSize;
}

void SX1276::setReceive()
{
	uint32_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS))
	{
		writeRegister(REG_OP_MODE, MODE_STDBY);
		writeRegister(REG_DIO_MAPPING_1, 0);
		// writeRegister(LR_RegHopPeriod, 0xFF);	//No FHSS
		writeRegister(LR_RegHopPeriod, 0x00); // No FHSS
		writeRegister(REG_PAYLOAD_LENGTH, getMessageSize());
		writeRegister(REG_FIFO_ADDR_PTR, 0);
		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE);
		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
	}
}

uint8_t SX1276::parsePacket()
{
	uint8_t packetLength = 0;
	loraStatus_e stat = getStatus();
	if (stat != tx && stat != disconnected)
	{
		writeRegister(REG_PAYLOAD_LENGTH, getMessageSize());

		uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
		// clear IRQ's
		writeRegister(REG_IRQ_FLAGS, irqFlags);

		if ((irqFlags & IRQ_RX_DONE_MASK) && ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0))
		{
			// received a packet
			_packetIndex = 0;

			// read packet length
			if (messageSize > 0)
			{
				packetLength = messageSize;
			}
			else
			{
				packetLength = readRegister(REG_RX_NB_BYTES);
			}

			// set FIFO address to current RX address
			writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

			// put in standby mode
			setIdle();
		}
		else if (getMode() != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
		{
			// not currently in RX mode

			// reset FIFO address
			writeRegister(REG_FIFO_ADDR_PTR, 0);

			// put in single RX mode
			writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
		}
		return packetLength;
	}
	return 0;
}

uint8_t SX1276::packetRssi()
{
	uint8_t reg;
	reg = readRegister(REG_PKT_RSSI_VALUE);
	return (reg - (_frequency < 868E6 ? 164 : 157));
}

float SX1276::packetSnr()
{
	uint8_t reg;
	reg = readRegister(REG_PKT_SNR_VALUE);
	return ((int8_t)reg) * 0.25;
}

uint8_t SX1276::available()
{
	loraStatus_e stat = getStatus();
	if (stat == tx || stat == disconnected)
		return 0;
	uint8_t reg;
	reg = readRegister(REG_RX_NB_BYTES);
	return (reg - _packetIndex);
}

int16_t SX1276::read()
{
	if (!available())
	{
		return -1;
	}

	_packetIndex++;

	uint8_t reg;
	reg = readRegister(REG_FIFO);

	return reg;
}

uint16_t SX1276::readBytes(uint8_t *buffer, uint16_t length)
{
	uint16_t i = 0;
	uint8_t data;
	messagePending = false;
	while ((data = read()) >= 0)
	{
		if (i == length)
			break;
		buffer[i] = data;
		i++;
	}
	return i;
}

int16_t SX1276::peek()
{
	if (!available())
	{
		return -1;
	}
	uint8_t b;
	// store current FIFO address
	int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

	// read
	b = readRegister(REG_FIFO);

	// restore FIFO address
	writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

	return b;
}

int8_t SX1276::getTemperature()
{
	int8_t temp;
	uint8_t regVal;
	regVal = readRegister(REG_TEMP);

	temp = regVal & 0x7F;
	if ((regVal & 0x80))
	{
		temp *= -1;
	}
	return temp;
}

void SX1276::setLoraMode()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE);
	writeRegister(REG_OP_MODE, MODE_STDBY);
}

void SX1276::setIdle()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void SX1276::setSleep()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void SX1276::setTxPower(uint8_t level)
{
	if (level < 2)
	{
		level = 2;
	}
	else if (level > 17)
	{
		level = 17;
	}
	writeRegisterSafe(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void SX1276::setFrequency(uint32_t frequency)
{
	_frequency = frequency;

	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
	uint8_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		setSleep();
	writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		writeRegister(REG_OP_MODE, mode);
}

void SX1276::setSpreadingFactor(uint8_t sf)
{
	if (sf < 6)
	{
		sf = 6;
	}
	else if (sf > 12)
	{
		sf = 12;
	}
	uint8_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		setSleep();
	mode = getMode();
	if (sf == 6)
	{
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
	}
	else
	{
		writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		writeRegister(REG_OP_MODE, mode);
}

void SX1276::setSignalBandwidth(uint32_t sbw)
{
	uint8_t bw = 9;

	if (sbw <= 7.8E3)
	{
		bw = 0;
	}
	else if (sbw <= 10.4E3)
	{
		bw = 1;
	}
	else if (sbw <= 15.6E3)
	{
		bw = 2;
	}
	else if (sbw <= 20.8E3)
	{
		bw = 3;
	}
	else if (sbw <= 31.25E3)
	{
		bw = 4;
	}
	else if (sbw <= 41.7E3)
	{
		bw = 5;
	}
	else if (sbw <= 62.5E3)
	{
		bw = 6;
	}
	else if (sbw <= 125E3)
	{
		bw = 7;
	}
	else if (sbw <= 250E3)
	{
		bw = 8;
	}
	else if (sbw <= 500E3)
	{
		bw = 9;
	}
	writeRegisterSafe(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void SX1276::setCodingRate4(uint8_t denominator)
{
	if (denominator < 5)
	{
		denominator = 5;
	}
	else if (denominator > 8)
	{
		denominator = 8;
	}

	uint8_t cr = denominator - 4;
	writeRegisterSafe(REG_MODEM_CONFIG_1,
					  (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void SX1276::setPreambleLength(uint16_t length)
{
	uint8_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		setSleep();
	writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		writeRegister(REG_OP_MODE, mode);
}

void SX1276::setSyncWord(uint8_t sw)
{
	writeRegisterSafe(REG_SYNC_WORD, sw);
}

uint8_t SX1276::getMode()
{
	uint8_t mode;
	mode = readRegister(REG_OP_MODE);
	return mode;
}

void SX1276::crc()
{
	writeRegisterSafe(REG_MODEM_CONFIG_2,
					  readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void SX1276::noCrc()
{
	writeRegisterSafe(REG_MODEM_CONFIG_2,
					  readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

uint8_t SX1276::random()
{
	uint8_t reg;
	reg = readRegister(REG_RSSI_WIDEBAND);
	return reg;
}

void SX1276::explicitHeaderMode()
{
	writeRegister(REG_MODEM_CONFIG_1,
				  readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void SX1276::implicitHeaderMode()
{
	writeRegister(REG_MODEM_CONFIG_1,
				  readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

bool SX1276::messageReceived()
{
	return messagePending;
}

