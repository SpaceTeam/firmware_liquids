#ifndef ECU_LAMARR_H
#define ECU_LAMARR_H

#include <Channels/ADCChannel.h>
#include <Channels/TempChannel.h>
#include <Channels/DigitalOutChannel.h>
#include <Channels/DigitalInChannel.h>
#include <Channels/PyroChannel.h>
#include <Channels/PressureControlChannel.h>
#include <Channels/ServoChannel.h>
#include <Channels/GenericChannel.h>
#include <Channels/RocketChannel.h>
#include <Can.h>
#include "../Modules/W25Qxx_Flash.h"
#include <Speaker.h>
#include <STRHAL.h>

class ECU_Lamarr: public GenericChannel
{
	public:
		ECU_Lamarr(uint32_t node_id, uint32_t fw_version, uint32_t refresh_divider);
		ECU_Lamarr(const ECU_Lamarr &other) = delete;
		ECU_Lamarr& operator=(const ECU_Lamarr &other) = delete;

		int init() override;
		//int reset() override;
		int exec() override;

		void testChannels();
		void testServo(ServoChannel &servo);

	private:



		STRHAL_GPIO_t led_1, led_2;

		// Channels
		ADCChannel press_0, press_1, press_2, press_3;
		TempChannel temp_0, temp_1;
		ServoChannel servo_0, servo_1;
		DigitalInChannel pyro0_cont, pyro1_cont, pyro2_cont, pyro3_cont;
		PyroChannel pyro_igniter0, pyro_igniter1, pyro_igniter2, pyro_igniter3;
		//PressureControlChannel pressure_control;
		//RocketChannel rocket;
		Speaker speaker;
		//Modules
		MAX31865_Temp max_temp_0, max_temp_1;
};

#endif /*ECU_UHB_H*/
