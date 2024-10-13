#ifndef ROCKETCHANNEL_H
#define ROCKETCHANNEL_H

#include "./Channels/AbstractChannel.h"
#include "./Channels/ADCChannel.h"
#include "./Channels/ServoChannel.h"
#include "./Channels/PyroChannel.h"
#include "./Channels/PIControlChannel.h"
#include <can_houbolt/channels/rocket_channel_def.h>
#include <STRHAL.h>
#include <STRHAL_UART.h>
#include <cstring>
#include <cstdio>

enum class IgnitionSequence : int
{
	INIT = 0,
	IGNITION_ON1,
	IGNITION_ON2,
	ENABLE_OX_PRESSURANT,
	OPEN_OX_MAIN,
	ENABLE_FUEL_PRESSURANT,
	OPEN_FUEL_MAIN,
	IGNITION_OFF
};

class RocketChannel: public AbstractChannel
{
	public:
		RocketChannel(uint8_t id, const ADCChannel &fuelPressureChannel,const ADCChannel &oxPressureChannel, const ADCChannel &chamberPressureChannel, ServoChannel &fuelServoChannel, ServoChannel &oxServoChannel, PIControlChannel &piControlChannel, PyroChannel &internalIgniter1Channel, PyroChannel &internalIgniter2Channel, PyroChannel &ventValveChannel, uint8_t is_main_ecu, uint32_t refreshDivider);
		RocketChannel(const RocketChannel &other) = delete;
		RocketChannel& operator=(const RocketChannel &other) = delete;
		RocketChannel(const RocketChannel &&other) = delete;
		RocketChannel& operator=(const RocketChannel &&other) = delete;

		int init() override;
		int reset() override;
		int exec() override;
		int getSensorData(uint8_t *data, uint8_t &n) override;

		int processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) override;

		static constexpr uint16_t EXEC_SAMPLE_TICKS = 1;
		static constexpr uint16_t HOLDDOWN_DELAY = 1000;
		static constexpr uint16_t CHAMBER_PRESSURE_LOW_PENALTY = 3;
		static constexpr uint16_t CHAMBER_PRESSURE_LOW_COUNT_MAX = 5;
		static constexpr uint16_t CHAMBER_PRESSURE_GOOD_COUNT_MIN = 100;
		static constexpr uint16_t FLIGHT_BURN_TIME = 4500;
		static constexpr uint16_t AUTO_CHECK_BAD_COUNT_MAX = 10;

	protected:

		int setVariable(uint8_t variableId, int32_t data) override;
		int getVariable(uint8_t variableId, int32_t &data) const override;

	private:
		ROCKET_STATE currentStateLogic(uint64_t time);
		void nextStateLogic(ROCKET_STATE nextState, uint64_t time);

		ROCKET_STATE autoCheck(uint64_t time);
		ROCKET_STATE ignitionSequence(uint64_t time);
		ROCKET_STATE holddown(uint64_t time);
		ROCKET_STATE poweredAscent(uint64_t time);
		ROCKET_STATE depress(uint64_t time);
		ROCKET_STATE abort(uint64_t time);
		ROCKET_STATE pressurize_tanks(uint64_t time);

		void setRocketState(uint8_t *data, uint8_t &n);
		void getRocketState(uint8_t *data, uint8_t &n);
		void SetRemoteRocketState(DeviceIds device_id, ROCKET_CMDs state);
		double GetSensorReading(const ADCChannel& sensor_channel);

		const ADCChannel &fuelPressureChannel;
		const ADCChannel &oxPressureChannel;
		const ADCChannel &chamberPressureChannel;
		ServoChannel &fuelServoChannel;
		ServoChannel &oxServoChannel;
		PIControlChannel &piControlChannel;
		PyroChannel &internalIgniter1Channel;
		PyroChannel &internalIgniter2Channel;
		PyroChannel &ventValveChannel;
		uint8_t is_main_ecu;
		ROCKET_STATE state;
		IgnitionSequence ignitionState;

	    double sensor_slope = 0.01888275146;
	    double sensor_offset = -15;
		double chamberPressureMin = 9.5;
		double fuelPressureMin = 0;
		double oxPressureMin = 0;

		uint16_t chamberPressureLowCounter = 0;
		uint16_t chamberPressureGoodCounter = 0;
		uint16_t autoCheckBadCounter = 0;
		uint16_t holdDownTimeout = 10000;
		uint64_t timeLastSample = 0;
		uint64_t timeLastTransition = 0;
		uint64_t timeSinceBothMainValvesOpen = 0;
		ROCKET_STATE internalNextState = PAD_IDLE;
		ROCKET_STATE externalNextState = PAD_IDLE;

		Can& can;
};

#endif /*ADCCHANNEL_H*/
