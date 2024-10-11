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
bool SX1276::singleTransfer(uint8_t address, uint8_t value)
{
	uint8_t recv;
	const uint8_t data[] = {address, value};
	return STRHAL_SPI_Master_Transceive(spiId, data, sizeof(data), 1, &recv, 1, 100);
}

bool SX1276::fifoTransfer(uint8_t address, const uint8_t *buffer, size_t len)
{
	uint8_t dummy;
	std::vector<uint8_t> buf;
	buf.push_back(address | 0x80);
	for (size_t i = 0; i < len; i++)
	{
		buf.push_back(buffer[i]);
	}
	return STRHAL_SPI_Master_Transceive(spiId, &buf[0], len, len, &dummy, 0, 100);
}

bool SX1276::readRegister(uint8_t address)
{
	return singleTransfer(address & 0x7f, 0x00);
}

bool SX1276::writeRegister(uint8_t address, uint8_t value)
{
	return singleTransfer(address | 0x80, value);
}

bool SX1276::writeRegisterSafe(uint8_t address, uint8_t value)
{
	bool ret = true;
	uint8_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= setSleep();
	ret &= writeRegister(address, value);
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= writeRegister(REG_OP_MODE, mode);
	return ret;
}


/**** Functions ****/
SX1276::SX1276(const STRHAL_SPI_Id_t &spiId, const STRHAL_SPI_Config_t &spiConf, const STRHAL_GPIO_t &dio0, const STRHAL_GPIO_t &reset) : spiId(spiId), spiConf(spiConf), dio0(dio0), reset(reset)
{

}

unsigned char SX1276::init(const loraSettings_t *settings)
{
	loraLocked = 0;

	memcpy(&currentSettings, settings, sizeof(loraSettings_t));
	ConfigureLora();

	return 1;
}

bool SX1276::ConfigureLora()
{
	bool ret = true;
	ret &= setSleep();
	ret &= setFrequency(currentSettings.frequency);
	ret &= writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	ret &= writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
	ret &= writeRegister(REG_OCP, 0x0B);
	ret &= writeRegister(REG_LNA, 0x23);
	ret &= writeRegister(0x36, 0x02); // See Errata note
	ret &= writeRegister(0x3A, 0x64); // See Errata note
	ret &= setTxPower(currentSettings.txPower);
	ret &= setSpreadingFactor(currentSettings.spreadingFactor);
	ret &= setCodingRate4(currentSettings.codingRateDenominator);
	ret &= setSignalBandwidth(currentSettings.signalBandwith);
	ret &= setPreambleLength(currentSettings.preambleLength);
	ret &= setSyncWord(currentSettings.syncword);
	ret &= setMessageSize(currentSettings.messageSize);
	if (currentSettings.crc)
	{
		ret &= crc();
	}
	else
	{
		ret &= noCrc();
	}

	ret &= setLoraMode();
	return ret;
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

bool SX1276::sendBytes(const uint8_t *buffer, uint8_t length)
{
	bool ret = true;
	uint8_t data[messageSize];
	if (length > messageSize)
	{
		return false;
	}
	for (uint8_t i = 0; i < messageSize; i++)
	{
		if (i < length)
		{
			data[i] = buffer[i];
		}
		else
		{
			data[i] = 0;
		}
	}
	ret &= setIdle();
	ret &= writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); // Clear irq
	ret &= writeRegister(LR_RegHopPeriod, 0x00);			// No FHSS
	ret &= writeRegister(REG_DIO_MAPPING_1, 1 << 6);
	ret &= writeRegister(REG_FIFO_ADDR_PTR, 0);
	ret &= fifoTransfer(REG_FIFO, data, messageSize);
	ret &= writeRegister(REG_PAYLOAD_LENGTH, messageSize);
	ret &= writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	uint32_t startTime = time_t();
	uint8_t val = readRegister(REG_IRQ_FLAGS);
	uint8_t val;
	if (!readRegister(REG_IRQ_FLAGS, val))
		return false;
	while ((val & IRQ_TX_DONE_MASK) == 0)
	{
		if ((time_t() - startTime) > 200)
		{
			return false;
		}
		if (!readRegister(REG_IRQ_FLAGS, val))
			return false;
	}
	ret &= writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	return ret;
}

bool SX1276::setMessageSize(uint8_t size)
{
	bool ret = true;
	if (size > 0)
	{
		ret &= implicitHeaderMode();
		ret &= writeRegister(REG_PAYLOAD_LENGTH, size);
	}
	else
	{
		ret &= explicitHeaderMode();
	}
	messageSize = size;
	return ret;
}

uint8_t SX1276::getMessageSize()
{
	return messageSize;
}

bool SX1276::setReceive()
{
	bool ret = true;
	uint32_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS))
	{
		ret &= writeRegister(REG_OP_MODE, MODE_STDBY);
		ret &= writeRegister(REG_DIO_MAPPING_1, 0);
		// ret &= writeRegister(LR_RegHopPeriod, 0xFF);	//No FHSS
		ret &= writeRegister(LR_RegHopPeriod, 0x00); // No FHSS
		ret &= writeRegister(REG_PAYLOAD_LENGTH, getMessageSize());
		ret &= writeRegister(REG_FIFO_ADDR_PTR, 0);
		ret &= writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE);
		ret &= writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
	}
	return ret;
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

bool SX1276::setLoraMode()
{
	bool ret = true;
	ret &= writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE);
	ret &= writeRegister(REG_OP_MODE, MODE_STDBY);
	return ret;
}

bool SX1276::setIdle()
{
	return writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

bool SX1276::setSleep()
{
	return writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

bool SX1276::setTxPower(uint8_t level)
{
	if (level < 2)
	{
		level = 2;
	}
	else if (level > 17)
	{
		level = 17;
	}
	return writeRegisterSafe(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

bool SX1276::setFrequency(uint32_t frequency)
{
	bool ret = true;
	_frequency = frequency;

	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
	uint8_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= setSleep();
	ret &= writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	ret &= writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	ret &= writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= writeRegister(REG_OP_MODE, mode);
	return ret;
}

bool SX1276::setSpreadingFactor(uint8_t sf)
{
	bool ret = true;
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
		ret &= setSleep();
	if (sf == 6)
	{
		ret &= writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		ret &= writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
	}
	else
	{
		ret &= writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		ret &= writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	ret &= writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= writeRegister(REG_OP_MODE, mode);
	return ret;
}

bool SX1276::setSignalBandwidth(uint32_t sbw)
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
	return writeRegisterSafe(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

bool SX1276::setCodingRate4(uint8_t denominator)
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
	return writeRegisterSafe(REG_MODEM_CONFIG_1,
					  (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

bool SX1276::setPreambleLength(uint16_t length)
{
	bool ret = true;
	uint8_t mode = getMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= setSleep();
	ret &= writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	ret &= writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= writeRegister(REG_OP_MODE, mode);
	return ret;
}

bool SX1276::setSyncWord(uint8_t sw)
{
	return writeRegisterSafe(REG_SYNC_WORD, sw);
}

uint8_t SX1276::getMode()
{
	uint8_t mode;
	mode = readRegister(REG_OP_MODE);
	return mode;
}

bool SX1276::crc()
{
	return writeRegisterSafe(REG_MODEM_CONFIG_2,
					  readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

bool SX1276::noCrc()
{
	return writeRegisterSafe(REG_MODEM_CONFIG_2,
					  readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

uint8_t SX1276::random()
{
	uint8_t reg;
	reg = readRegister(REG_RSSI_WIDEBAND);
	return reg;
}

bool SX1276::explicitHeaderMode()
{
	return writeRegister(REG_MODEM_CONFIG_1,
				  readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

bool SX1276::implicitHeaderMode()
{
	return writeRegister(REG_MODEM_CONFIG_1,
				  readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

bool SX1276::messageReceived()
{
	return messagePending;
}
