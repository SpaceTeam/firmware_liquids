#include "../../Inc/Modules/LoRa1276F30_Radio.h"
#include <stm32g4xx_ll_exti.h>
#include <stm32g4xx_ll_system.h>

#include <cstring>
#include <cstdio>
#include <array>
#include <ctime>
#include <time.h>

LoRa1276F30_Radio::LoRa1276F30_Radio(const STRHAL_SPI_Id_t &spiId, const STRHAL_SPI_Config_t &spiConf, const STRHAL_GPIO_t &dio0, const STRHAL_GPIO_t &reset) : spiId(spiId), spiConf(spiConf), dio0(dio0), reset(reset)
{
}

int LoRa1276F30_Radio::init()
{
	// Uncomment if baro interrupt should be used
	/*LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	 EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
	 EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
	 EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	 EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	 EXTI_InitStruct.LineCommand = ENABLE;
	 if(LL_EXTI_Init(&EXTI_InitStruct) != 0)
	 return -1;

	 LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE1);

	 NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 1));
	 NVIC_EnableIRQ(EXTI1_IRQn);*/

	STRHAL_GPIO_SingleInit(&dio0, STRHAL_GPIO_TYPE_IHZ);
	STRHAL_GPIO_SingleInit(&reset, STRHAL_GPIO_TYPE_OPP);

	if (STRHAL_SPI_Master_Init(spiId, &spiConf) < 0)
		return -1;

	STRHAL_SPI_Master_Run(spiId);
	LL_mDelay(10);
	resetFunc();

	return Configure();
}

int LoRa1276F30_Radio::Configure()
{
	bool ret = true;
	ret &= SetSleep();
	ret &= SetFrequency(868e6);
	ret &= lora_writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
	ret &= lora_writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
	ret &= lora_writeRegister(REG_OCP, 0x0B);
	ret &= lora_writeRegister(REG_LNA, 0x23);
	ret &= lora_writeRegister(0x36, 0x02); // See Errata note
	ret &= lora_writeRegister(0x3A, 0x64); // See Errata note
	ret &= SetTxPower(17);
	ret &= SetSpreadingFactor(SF10);
	ret &= SetCodingRate4(6);
	ret &= SetSignalBandwidth(BW500);
	ret &= SetPreambleLength(8);
	ret &= SetSyncWord(0xE4);
	ret &= lora_explicitHeaderMode();
	//ret &= imageCalibration();
	ret &= lora_writeRegister(REG_PAYLOAD_LENGTH, PKT_LENGTH);
	//ret &= EnableCRC();
	//ReadVersion();

	return ret;
}

bool LoRa1276F30_Radio::imageCalibration() const
{
	if(!lora_writeRegister(REG_ImageCal, 1 << 6)) return false;
	uint8_t val = 0;
	uint32_t startTime = time_t();
	do
	{
		if ((time_t() - startTime) > 200)
		{
			return false;
		}
		if(!lora_readRegister(REG_ImageCal, val)) return false;
	} while(val & (1 << 5));
	return true;
}

uint8_t LoRa1276F30_Radio::ReadVersion() const
{
	uint8_t reg;
	bool ret = lora_readRegister(REG_VERSION, reg);
	return ret ? reg : 0;
}

bool LoRa1276F30_Radio::SetLoraMode()
{
	bool ret = true;
	ret &= lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE);
	ret &= lora_writeRegister(REG_OP_MODE, MODE_STDBY);
	return ret;
}

bool LoRa1276F30_Radio::SetIdle() const
{
	return lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

bool LoRa1276F30_Radio::SetSleep() const
{
	return lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

uint8_t LoRa1276F30_Radio::GetMode() const
{
	uint8_t mode;
	bool ret = lora_readRegister(REG_OP_MODE, mode);
	return ret ? mode : 0;
}

bool LoRa1276F30_Radio::SetFrequency(uint32_t frequency)
{
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
	uint8_t mode = GetMode();
	bool ret = true;
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= SetSleep();
	ret &= lora_writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	ret &= lora_writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	ret &= lora_writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= lora_writeRegister(REG_OP_MODE, mode);
	return ret;
}

bool LoRa1276F30_Radio::SetTxPower(uint8_t level)
{
	if (level < 2)
	{
		level = 2;
	}
	else if (level > 17)
	{
		level = 17;
	}
	return lora_writeRegisterSafe(REG_PA_CONFIG, static_cast<uint8_t>(PA_BOOST | (level - 2)));
}

bool LoRa1276F30_Radio::SetSpreadingFactor(const spreadingFactor_t &sf)
{
	uint8_t sfNumber = static_cast<uint8_t>(sf);
	uint8_t mode = GetMode();
	bool ret = true;
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= SetSleep();
	if (sfNumber == 6)
	{
		ret &= lora_writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		ret &= lora_writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
	}
	else
	{
		ret &= lora_writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		ret &= lora_writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	ret &= lora_writeRegister(REG_MODEM_CONFIG_2, static_cast<uint8_t>((lora_readRegister(REG_MODEM_CONFIG_2, sfNumber) & 0x0f) | ((sfNumber << 4) & 0xf0)));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= lora_writeRegister(REG_OP_MODE, mode);
	return ret;
}

bool LoRa1276F30_Radio::SetSignalBandwidth(const bandwith_t &sbw)
{
	uint8_t bw = static_cast<uint8_t>(sbw);
	return lora_writeRegisterSafe(REG_MODEM_CONFIG_1, static_cast<uint8_t>((lora_readRegister(REG_MODEM_CONFIG_1, bw) & 0x0f) | (bw << 4)));
}

bool LoRa1276F30_Radio::SetCodingRate4(uint8_t denominator)
{
	if (denominator < 5)
	{
		denominator = 5;
	}
	else if (denominator > 8)
	{
		denominator = 8;
	}

	uint8_t dummy;
	uint8_t cr = denominator - 4;
	lora_writeRegisterSafe(REG_MODEM_CONFIG_1,
					  (lora_readRegister(REG_MODEM_CONFIG_1, dummy) & 0xf1) | (cr << 1));
	return true;
}

bool LoRa1276F30_Radio::SetPreambleLength(uint16_t length)
{
	bool ret = true;
	uint8_t mode = GetMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= SetSleep();
	ret &= lora_writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	ret &= lora_writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		ret &= lora_writeRegister(REG_OP_MODE, mode);
	return ret;
}

bool LoRa1276F30_Radio::SetSyncWord(uint8_t sw)
{
	return lora_writeRegisterSafe(REG_SYNC_WORD, sw);
}

bool LoRa1276F30_Radio::EnableCRC()
{
	uint8_t val;
	if (!lora_readRegister(REG_MODEM_CONFIG_2, val))
		return false;
	return lora_writeRegisterSafe(REG_MODEM_CONFIG_2,
								  val | 0x04);
}

bool LoRa1276F30_Radio::DisableCRC()
{
	uint8_t val;
	if (!lora_readRegister(REG_MODEM_CONFIG_2, val))
		return false;
	return lora_writeRegisterSafe(REG_MODEM_CONFIG_2,
								  val & 0xfb);
}

int LoRa1276F30_Radio::resetFunc()
{
	STRHAL_GPIO_Write(&reset, STRHAL_GPIO_VALUE_L);
	LL_mDelay(1);
	STRHAL_GPIO_Write(&reset, STRHAL_GPIO_VALUE_H);

	return 0;
}

bool LoRa1276F30_Radio::sendBytes(uint8_t *buffer, uint8_t n)
{
	uint8_t data[PKT_LENGTH];
	if (n > PKT_LENGTH)
	{
		return false;
	}
	for (uint8_t i = 0; i < PKT_LENGTH; i++)
	{
		if (i < n)
		{
			data[i] = buffer[i];
		}
		else
		{
			data[i] = 0;
		}
	}
	bool ret = true;
	ret &= SetIdle();
	ret &= lora_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); // Clear irq
	ret &= lora_writeRegister(LR_RegHopPeriod, 0x00);			// No FHSS
	ret &= lora_writeRegister(REG_DIO_MAPPING_1, 1 << 6);
	ret &= lora_writeRegister(REG_FIFO_ADDR_PTR, 0);
	lora_fifoTransfer(REG_FIFO, data, PKT_LENGTH);
	ret &= lora_writeRegister(REG_PAYLOAD_LENGTH, PKT_LENGTH);
	ret &= lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	uint32_t startTime = time_t();
	uint8_t val;
	if (!lora_readRegister(REG_IRQ_FLAGS, val))
		return false;
	while ((val & IRQ_TX_DONE_MASK) == 0)
	{
		if ((time_t() - startTime) > 200)
		{
			return false;
		}
		if (!lora_readRegister(REG_IRQ_FLAGS, val))
			return false;
	}
	ret &= lora_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	return ret;
}

bool LoRa1276F30_Radio::lora_singleTransfer(uint8_t address, uint8_t value, uint8_t &received) const
{
	const uint8_t data[] = {address, value};
	return STRHAL_SPI_Master_Transceive(spiId, data, sizeof(data), 1, &received, 1, 100) > 0;
}

bool LoRa1276F30_Radio::lora_fifoTransfer(uint8_t address, const uint8_t *buffer, size_t length) const
{
	uint8_t dummy;
	return STRHAL_SPI_Master_Transceive(spiId, buffer, length, length, &dummy, 0, 100) > 0;
}

bool LoRa1276F30_Radio::lora_readRegister(uint8_t address, uint8_t &received) const
{
	return lora_singleTransfer(address & 0x7f, 0x00, received);
}

bool LoRa1276F30_Radio::lora_writeRegister(uint8_t address, uint8_t value) const
{
	uint8_t received;
	return lora_singleTransfer(address | 0x80, value, received);
}

bool LoRa1276F30_Radio::lora_writeRegisterSafe(uint8_t address, uint8_t value)
{
	uint8_t mode = GetMode();
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		SetSleep();
	bool ret = lora_writeRegister(address, value);
	if (mode != (MODE_LONG_RANGE_MODE | MODE_SLEEP))
		lora_writeRegister(REG_OP_MODE, mode);
	return ret;
}

bool LoRa1276F30_Radio::lora_explicitHeaderMode() const
{
	uint8_t value;
	if(!lora_readRegister(REG_MODEM_CONFIG_1, value)) return false;
	return lora_writeRegister(REG_MODEM_CONFIG_1, value & 0xfe);
}

bool LoRa1276F30_Radio::lora_implicitHeaderMode() const
{
	uint8_t value;
	if(!lora_readRegister(REG_MODEM_CONFIG_1, value)) return false;
	return lora_writeRegister(REG_MODEM_CONFIG_1, value | 0x01);
}
