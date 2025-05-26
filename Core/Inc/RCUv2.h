#ifndef RCU_V2_H
#define RCU_V2_H

#include <Channels/ADCChannel.h>
#include <Channels/Data32Channel.h>
#include <Channels/DigitalOutChannel.h>
#include <Channels/DigitalInChannel.h>
#include <Channels/GenericChannel.h>
#include <Channels/BaroChannel.h>
#include <Can.h>
#include <Radio.h>
#include <Channels/IMUChannel.h>
#include "../Modules/W25Qxx_Flash.h"
#include "../Modules/SX1276.h"
#include "../Modules/SAM_M8Q_GNSS.h"
#include <Speaker.h>

#include <STRHAL.h>

class RCUv2: public GenericChannel
{
	public:
		RCUv2(uint32_t node_id, uint32_t fw_version, uint32_t refresh_divider);
		RCUv2(const RCUv2 &other) = delete;
		RCUv2& operator=(const RCUv2 &other) = delete;

		int init() override;
		//int reset() override;
		int exec() override;
		void testIMU();
		void testGNSS();
		void beep(int frequency, int length, int delay);
		void startupBeep();
		void superMario();
		void gnssBeep();

	private:
		STRHAL_GPIO_t led1, led2;

		// Modules
		LPS25HB_Baro baro;
		ICM2060x_IMU imu;
		SX1276 lora;
		SAM_M8Q_GNSS gnss;

		// Channels
		ADCChannel sense_5V, sense_12V;
		BaroChannel baro_channel;
		IMUChannel x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro;
		Data32Channel gps_longitude, gps_latitude, gps_altitude, gps_status;
		DigitalOutChannel out0,out1,out2,out3;

		// Coms
		Radio &radio;

		Speaker speaker;
};

#endif /*RCU_V2_H*/
