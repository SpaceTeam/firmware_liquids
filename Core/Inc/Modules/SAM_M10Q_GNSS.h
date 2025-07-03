#ifndef SAM_M10Q_GNSS_H
#define SAM_M10Q_GNSS_H

#include <STRHAL.h>
#include "../Ubx.h"
#include "./Modules/AbstractModule.h"

//OLD
#define UBLOX_SYNC1     0xB5		// Checked
#define UBLOX_SYNC2     0x62		// Checked

#define UBLOX_NAV_CLASS     0x01	// Checked
#define UBLOX_RXM_CLASS     0x02	// Checked
#define UBLOX_CFG_CLASS     0x06	// Checked
#define UBLOX_MON_CLASS     0x0a	// Checked
#define UBLOX_AID_CLASS     0x0b	// DEPRECATED	-> Use MGA class instead
#define UBLOX_TIM_CLASS     0x0d	// Checked

//NEW
#define UBLOX_MGA_CLASS		0x13


//OLD
#define UBLOX_NAV_POSLLH    0x02	// Checked
#define UBLOX_NAV_STATUS    0x03	// Checked
#define UBLOX_NAV_DOP       0x04	// Checked
#define UBLOX_NAV_VELNED    0x12	// Checked
#define UBLOX_NAV_TIMEUTC   0x21	// Checked
#define UBLOX_NAV_SBAS      0x32	// Checked
//#define UBLOX_NAV_SOL       0x06	// DEPRECATED	-> Use NAV-PVT 0x07 instead
//#define UBLOX_NAV_SVINFO    0x30	// DEPRECATED	-> NAV-SIG 0x43 & NAV-SAT 0x35

//NEW
#define UBLOX_NAV_PVT		0x07
#define UBLOX_NAV_SAT		0x35
#define UBLOX_NAV_SIG		0x43

//OLD
//#define UBLOX_AID_REQ       0x00	// DEPRECATED	-> Is not referenced anywhere, DELETE

#define UBLOX_RXM_SFRB      0x13	// Checked
//#define UBLOX_RXM_RAW       0x10	// DEPRECATED	-> Is not referenced anywhere, DELETE

#define UBLOX_MON_VER       0x04	// Checked
//#define UBLOX_MON_HW        0x09	// DEPRECATED	-> Is not referenced anywhere, Use MON_HW3 0x37 instead

//NEW
#define MON_HW3 0x37

//OLD
#define UBLOX_TIM_TP        0x01	// Checked

//#define UBLOX_CFG_MSG       0x01	// DEPRECATED
//#define UBLOX_CFG_TP        0x07	// DEPRECATED
//#define UBLOX_CFG_RATE      0x08	// DEPRECATED
//#define UBLOX_CFG_SBAS      0x16	// DEPRECATED
//#define UBLOX_CFG_NAV5      0x24	// DEPRECATED
//#define UBLOX_CFG_GNSS      0x3E	// DEPRECATED

#define UBLOX_CFG_CFG       0x09	// Checked
//NEW
#define UBLOX_CFG_RST 		0x04	// Checked
#define UBLOX_CFG_VALDEL 	0x8c	// Checked
#define UBLOX_CFG_VALGET 	0x8b	// Checked
#define UBLOX_CFG_VALSET 	0x8a	// Checked

//OLD
#define UBLOX_SBAS_AUTO     0x00000000
// TODO: Reverify these constants-- seems they have more bits set than they
// should.
#define UBLOX_SBAS_WAAS     0x0004E004
#define UBLOX_SBAS_EGNOS    0x00000851
#define UBLOX_SBAS_MSAS     0x00020200
#define UBLOX_SBAS_GAGAN    0x00000108

#define UBLOX_DYN_PORTABLE   0
#define UBLOX_DYN_STATIONARY 2
#define UBLOX_DYN_PED        3
#define UBLOX_DYN_AUTOMOTIVE 4
#define UBLOX_DYN_SEA        5
#define UBLOX_DYN_AIR1G      6
#define UBLOX_DYN_AIR2G      7
#define UBLOX_DYN_AIR4G      8

#define UBLOX_GNSSID_GPS     0
#define UBLOX_GNSSID_SBAS    1
#define UBLOX_GNSSID_GALILEO    2
#define UBLOX_GNSSID_BEIDOU  3
#define UBLOX_GNSSID_QZSS    5
#define UBLOX_GNSSID_GLONASS 6

#define UBLOX_MAX_PAYLOAD   384
#define UBLOX_WAIT_MS       100

enum class GNSSDynamicsMode : uint8_t
{
	PORTABLE,
	PEDESTRIAN,
	AUTOMOTIVE,
	AIRBORNE1G,
	AIRBORNE2G,
	AIRBORNE4G
};

enum class GNSSSbasConstellation : uint8_t
{
	WAAS,
	EGNOS,
	MSAS,
	GAGAN,
	ALL,
	NONE
};

enum class GNSSConstellation : uint8_t
{
	GLONASS,
	ALL,
	GPS,
	GALILEO,
	BEIDOU
};

enum class GNSSBaudRate : uint8_t
{
	_2400,
	_4800,
	_9600,
	_19200,
	_38400,
	_57600,
	_115200,
	_230400
};

typedef struct
{
		int32_t longitude;
		int32_t latitude;
		int32_t altitude;
		int32_t status;
} GNSSData_t;

constexpr static uint8_t ublox_request_baud[] = {
		 0xb5, 0x62,				// SYNC
		 0x06,						// class
		 0x00,						// id
		 0x14, 0x00,				// length

		 0x01, 0x00, 0x00, 0x00,	// portID = 1 (UART1)
		 0xd0, 0x08, 0x00, 0x00,	// mode (8N1, no parity)
		 0x80, 0x25, 0x00, 0x00, 	// BAUD 115200=1C200, 9600 = 2580
		 0x07, 0x00, 				// inProtoMask
		 0x07, 0x00,				// outProtoMask
		 0x00, 0x00, 0x00, 0x00,	// flags

		 0xc4, 0x96,				// checksum


		 0xb5, 0x62,				// SYNC
		 0x06,						// class
		 0x00,						// id
		 0x01, 0x00,				// length
		 0x01,						// payload
		 0x08, 0x22					// checksum//*/
};

// Ublox config: 5Hz update and 4g airborne
constexpr static uint8_t ublox_config[] = {
		0xB5, 0x62, 	// SYNC
		0x06, 			// class 	CFG
		0x8A, 			// id 		VALSET
		0x14, 0x00, 	// length

		0x01, 0x03, 0x00, 0x00,					// header
		0x57, 0x00, 0xA3, 0x20,  0x00, 			// CFG-HW-RF_LNA_MODE 	0 	= Normal
		0x01, 0x00, 0x21, 0x30,  0xC8, 0x00,   	// CFG-RATE-MEAS		c8 	= 200ms = 5Hz
		0x21, 0x00, 0x11, 0x20,  0x08,			// CFG-NAVSPG-DYNMODEL	8 	= Airborne with <4g acceleration

		0x34, 0x1B		// Checksum
};

constexpr static uint8_t ublox_valset[] = {
	0x06, 			// class 	CFG
	0x8A,			// id 		VALSET
	0x1D, 0x00,  	// Length = 60 bytes

	// Header
	0x01, 0x03, 0x00, 0x00,              // version=1, RAM layer

	// UBX-NAV-PVT enable on UART1
	//0x07, 0x00, 0x91, 0x20,  0x01, 0x00, 0x00, 0x00,	// 0x2091002a NAV-PVT

	0x2A, 0x00, 0x91, 0x20,  0x01,// 0x2091002a NAV-POSLLH
	0x07, 0x00, 0x91, 0x20,  0x01,// 0x20910007 NAV-PVT
	0x39, 0x00, 0x91, 0x20,  0x01,// 0x20910039 NAV-DOP
	0x43, 0x00, 0x91, 0x20,  0x01,// 0x20910043 NAV-VELNED
	0x5c, 0x00, 0x91, 0x20,  0x01,// 0x2091005c NAV-TIMEUTC//*/

	// Disable GGA–VTG (NMEA)
	//0x01, 0x00, 0x91, 0x20,  0x00, 0x00, 0x00, 0x00,
	//0x02, 0x00, 0x91, 0x20,  0x00, 0x00, 0x00, 0x00,
	//0x03, 0x00, 0x91, 0x20,  0x00, 0x00, 0x00, 0x00,
	//0x04, 0x00, 0x91, 0x20,  0x00, 0x00, 0x00, 0x00,
	//0x05, 0x00, 0x91, 0x20,  0x00, 0x00, 0x00, 0x00,
	//0x06, 0x00, 0x91, 0x20,  0x00, 0x00, 0x00, 0x00,
};

class SAM_M10Q_GNSS: public AbstractModule
{
	public:
		SAM_M10Q_GNSS(const STRHAL_UART_Id_t uartId, const STRHAL_GPIO_t &resetPin);

		int init() override;
		int exec() override;
		int reset() override;

		int processData(uint8_t *buffer, uint32_t length);

		int sendConfiguration(GNSSConstellation constellation, GNSSSbasConstellation sbas, GNSSDynamicsMode mode);

		GPSPositionData position =
		{ .Status = GPSPOSITION_STATUS_NOFIX };
		GNSSData_t gnssData =
		{ 0 };
	private:
		void resetChecksum();
		void updateChecksum(uint8_t c);

		int sendConfigDataChecksummed(const uint8_t *data, uint16_t length, uint32_t retries);
		int waitForACK(uint32_t delay);
		//int enableMessage(uint8_t msgClass, uint8_t msgId, uint8_t rate);
		int gnssSetup();
		int pollVersion();
		int clearConfig();
		//void setBaudrate(uintptr_t gps_port, GNSSBaudRate baud)

		const STRHAL_UART_Id_t uartId;
		STRHAL_GPIO_t resetPin;
		uint8_t checksumTxA = 0;
		uint8_t checksumTxB = 0;

		char gps_rx_buff[sizeof(struct UBXPacket)];
		struct GPS_RX_STATS rx_stats =
		{ 0 };

};

#endif /*SAM_M8Q_GNSS_H*/
