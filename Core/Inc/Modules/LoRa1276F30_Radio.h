#ifndef LORA1276F30_RADIO_H
#define LORA1276F30_RADIO_H

#include <STRHAL.h>
#include <cstdio>

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_PKT_SNR_VALUE 0x1b
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42
#define REG_TEMP 0x3C
#define REG_OCP 0x0B
#define LR_RegHopPeriod 0x24
#define REG_ImageCal 0x3b

// modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

typedef enum
{
	SF6 = 6,
	SF7,
	SF8,
	SF9,
	SF10,
	SF11,
	SF12
} spreadingFactor_t;

typedef enum
{
	BW7_8,
	BW10_4,
	BW15_6,
	BW20_8,
	BW31_25,
	BW41_7,
	BW62_5,
	BW125,
	BW250,
	BW500
} bandwith_t;

typedef enum
{
	CR5_8 = 5,
	CR6_8,
	CR7_8,
	CR8_8
} codingRate_t;

typedef struct
{
	uint32_t Frequency;
	uint8_t TxPower;
	spreadingFactor_t sf;
	codingRate_t cr;
	bandwith_t bw;
	uint16_t PreambleLength;
	uint8_t Syncword;
	bool EnableCRC;
} loraSetting;

class LoRa1276F30_Radio
{
public:
	LoRa1276F30_Radio(const STRHAL_SPI_Id_t &spiId, const STRHAL_SPI_Config_t &spiConf, const STRHAL_GPIO_t &dio0, const STRHAL_GPIO_t &reset);
	// LPS25HB_Baro(const LPS25HB_Baro &other) = delete;
	// LPS25HB_Baro& operator=(const LPS25HB_Baro &other) = delete;

	int init();
	int resetFunc();
	uint8_t ReadVersion() const;

	bool sendBytes(uint8_t *buffer, uint8_t n);
	static constexpr uint32_t PKT_LENGTH = 95; // 143
private:
	int Configure();
	bool SetLoraMode();
	bool SetIdle() const;
	bool SetSleep() const;
	uint8_t GetMode() const;
	bool SetFrequency(uint32_t frequency);
	bool SetTxPower(uint8_t level);
	bool SetSpreadingFactor(const spreadingFactor_t &sf);
	bool SetSignalBandwidth(const bandwith_t &sbw);
	bool SetCodingRate(const codingRate_t &codingrate);
	bool SetPreambleLength(uint16_t length);
	bool SetSyncWord(uint8_t sw);
	bool EnableCRC();
	bool DisableCRC();
	bool lora_singleTransfer(uint8_t address, uint8_t value, uint8_t &received) const;
	bool lora_fifoTransfer(uint8_t address, const uint8_t *buffer, size_t length) const;
	bool lora_readRegister(uint8_t address, uint8_t &received) const;
	bool lora_writeRegister(uint8_t address, uint8_t value) const;
	bool lora_writeRegisterSafe(uint8_t address, uint8_t value);
	bool lora_explicitHeaderMode() const;
	bool lora_implicitHeaderMode() const;
	bool imageCalibration() const;

	STRHAL_SPI_Id_t spiId;
	STRHAL_SPI_Config_t spiConf;
	const STRHAL_GPIO_t dio0;
	const STRHAL_GPIO_t reset;
};

#endif /*LORA1276F30_RADIO*/
