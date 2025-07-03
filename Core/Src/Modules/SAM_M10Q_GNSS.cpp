#include "../../Inc/Modules/SAM_M10Q_GNSS.h"
#include <stm32g4xx_ll_exti.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_usart.h>

#include <cstring>
#include <cstdio>

SAM_M10Q_GNSS::SAM_M10Q_GNSS(const STRHAL_UART_Id_t uartId, const STRHAL_GPIO_t &resetPin) :
		uartId(uartId), resetPin(resetPin)
{
}

int SAM_M10Q_GNSS::init()
{
	// init reset GPIO
	STRHAL_GPIO_SingleInit(&resetPin, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_Write(&resetPin, STRHAL_GPIO_VALUE_H);

	reset();

	// init GNSS uart
	if (STRHAL_UART_Instance_Init(uartId) != 0)
		return -1;

	LL_mDelay(1);

	STRHAL_UART_Listen(uartId);

	if(gnssSetup() != 0){
		return -1;
	}

	return 0;
}

int SAM_M10Q_GNSS::exec()
{
	uint8_t gnssBuf[64] =
	{ 0 };

	int32_t gnssRet = STRHAL_UART_Read(STRHAL_UART1, (char*) gnssBuf, 64);
	if (gnssRet > 0)
	{
		//STRHAL_UART_Debug_Write_Blocking((char*) gnssBuf, strlen((char*) gnssBuf), 100);
		processData(gnssBuf, gnssRet);
	}
	return 0;
}

int SAM_M10Q_GNSS::reset()
{
	STRHAL_GPIO_Write(&resetPin, STRHAL_GPIO_VALUE_L);
	LL_mDelay(1000);
	STRHAL_GPIO_Write(&resetPin, STRHAL_GPIO_VALUE_H);
	return 0;
}

int SAM_M10Q_GNSS::processData(uint8_t *buffer, uint32_t length)
{
	uint32_t id;
	uint8_t ret = 0;
	for (uint32_t i = 0; i < length; i++)
	{
		if (Ubx::parseStream(buffer[i], gps_rx_buff, &position, &rx_stats, &id) == PARSER_COMPLETE)
		{
			//char buf[64] = { 0 };
			//sprintf(buf, "GPS: %ld %d\n", id, position.Status);
			//STRHAL_UART_Debug_Write_Blocking(buf, strlen(buf), 200);
			if (position.Satellites < 0)
			{
				position.Satellites = 0;
			}
			// Check if we received a valid position solution and the solution is valid
			if (id == UBX_ID_POSLLH)
			{
				if (position.Status > GPSPOSITION_STATUS_NOFIX)
				{
					//sprintf(buf,"POS: %ld %ld\n",position.Longitude, position.Latitude);
					//STRHAL_UART_Debug_Write_Blocking(buf, strlen(buf), 200);
					//STRHAL_UART_Debug_Write_Blocking("FIX!!!\n", 7, 100);
					// Update the timestamp for the last valid solution
					//status.not_stored.body.gnss_last_fix_timestamp = status.main.body.timestamp;
					gnssData.longitude = position.Longitude;
					gnssData.latitude = position.Latitude;
					gnssData.altitude = position.Altitude;
					gnssData.status = position.Status;
					ret = 1;
				}
				else
				{
					// NOFIX
				}
			}
		}
	}
	return ret;
}

int SAM_M10Q_GNSS::gnssSetup() {

	//STRHAL_UART_Write_Blocking(uartId, (const char*) ublox_request_baud, sizeof(ublox_request_baud), 100);

	LL_mDelay(200);

	STRHAL_UART_Write_Blocking(uartId, (const char*) ublox_config, sizeof(ublox_config), 100);

	LL_mDelay(200);


	//STRHAL_UART_Write_Blocking(uartId, (const char*) ublox_valset, sizeof(ublox_valset), 100);

	int a = sendConfigDataChecksummed(ublox_valset, sizeof(ublox_valset), 5);

	if (a != 1)
	{
		return 1;
	}//*/

	STRHAL_UART_FlushReception(uartId);
	STRHAL_UART_Listen(uartId);
	return 0;
}

void SAM_M10Q_GNSS::resetChecksum()
{
	checksumTxA = 0;
	checksumTxB = 0;
}

void SAM_M10Q_GNSS::updateChecksum(uint8_t c)
{
	checksumTxA += c;
	checksumTxB += checksumTxA;
}

int SAM_M10Q_GNSS::sendConfigDataChecksummed(const uint8_t *data, uint16_t length, uint32_t retries)
{
	// Calculate checksum
	resetChecksum();
	for (uint16_t i = 0; i < length; i++)
	{
		updateChecksum(data[i]);
	}

	// Send buffer followed by checksum
	const uint8_t syncword[] =
	{ UBLOX_SYNC1, UBLOX_SYNC2 };
	const uint8_t checksum[] =
	{ checksumTxA, checksumTxB };

	for (uint32_t i = 0; i < retries; i++)
	{
		STRHAL_UART_Write_Blocking(uartId, (const char*) syncword, sizeof(syncword), 50);
		STRHAL_UART_Write_Blocking(uartId, (const char*) data, length, 50);
		STRHAL_UART_Write_Blocking(uartId, (const char*) checksum, sizeof(checksum), 50);

		if (waitForACK(1000) == 1)
		{
			return 1;
		}
		STRHAL_UART_FlushReception(uartId);
		STRHAL_UART_Listen(uartId);
	}
	return 0;
}

int SAM_M10Q_GNSS::waitForACK(uint32_t delay)
{
	int ret = PARSER_INCOMPLETE;
	struct GPS_RX_STATS gpsRxStats;
	GPSPositionData gpsPosition;

	uint8_t c;
	uint64_t enterTime = STRHAL_Systick_GetTick();
	while ((STRHAL_Systick_GetTick() - enterTime) < delay)
	{
		uint16_t received = STRHAL_UART_Read(uartId, (char*) &c, 1);
		if (received > 0)
		{
			//char buf[32] = { 0 };
			ret = Ubx::parseStream(c, gps_rx_buff, &gpsPosition, &gpsRxStats, 0);
			//sprintf(buf,"%d %d\n", c, ret);
			//STRHAL_UART_Debug_Write_Blocking(buf, strlen(buf), 100);
			switch (ret)
			{
				case PARSER_COMPLETE_ACK:
					return 1;
				case PARSER_COMPLETE_NAK:
					return 0;
				default:
					break;
			}
		}
	}

	return -1;     // Timeout
}
/*
int SAM_M10Q_GNSS::enableMessage(uint8_t msgClass, uint8_t msgId, uint8_t rate)
{
	const uint8_t msg[] =
	{
	UBLOX_CFG_CLASS,       // CFG
			UBLOX_CFG_MSG,         // MSG
			0x03,                  // length lsb
			0x00,                  // length msb
			msgClass,              // class
			msgId,                 // id
			rate,                  // rate
			};
	return sendConfigDataChecksummed(msg, sizeof(msg), 5);
}
*/








int SAM_M10Q_GNSS::pollVersion()
{
	const uint8_t msg[] =
	{ UBLOX_MON_CLASS, UBLOX_MON_VER, 0x00, 0x00 };
	return sendConfigDataChecksummed(msg, sizeof(msg), 5);
}

