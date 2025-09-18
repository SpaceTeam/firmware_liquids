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
#include <Speaker.h>

class RocketChannel: public AbstractChannel
{
	public:
		RocketChannel(
			uint8_t id, const ADCChannel &fuelPressureChannel, const ADCChannel &oxPressureChannel,
			const ADCChannel &chamberPressureChannel, ServoChannel &fuelServoChannel, ServoChannel &oxServoChannel,
			PIControlChannel &piControlChannel, PyroChannel &internalIgniter1Channel,
			PyroChannel &internalIgniter2Channel, PyroChannel &ventValveChannel,
			Speaker &speaker,
			uint32_t refreshDivider
		);
		RocketChannel(const RocketChannel &other) = delete;
		RocketChannel& operator=(const RocketChannel &other) = delete;
		RocketChannel(const RocketChannel &&other) = delete;
		RocketChannel& operator=(const RocketChannel &&other) = delete;

		int init() override;
		int reset() override;
		int exec() override;
		int getSensorData(uint8_t *data, uint8_t &n) override;

		int processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) override;

	protected:

		int setVariable(uint8_t variableId, int32_t data) override;
		int getVariable(uint8_t variableId, int32_t &data) const override;

	private:
		static constexpr uint16_t EXEC_SAMPLE_TICKS = 1;
		static constexpr uint16_t CHAMBER_PRESSURE_LOW_PENALTY = 3;
		static constexpr uint16_t CHAMBER_PRESSURE_LOW_COUNT_MAX = 5;
		static constexpr uint16_t CHAMBER_PRESSURE_GOOD_COUNT_MIN = 100;
		static constexpr uint16_t AUTO_CHECK_BAD_COUNT_MAX = 10;
		
		static constexpr uint16_t FLIGHT_BURN_TIME = 15000;
		static constexpr uint16_t HOLDDOWN_DELAY = 200;
		static constexpr uint16_t IGNITION_DELAY = 100;

		static constexpr u_int32_t GSE_CONNECTION_ABORT_MESSAGE_TIMEOUT = 300000;

		ROCKET_STATE nextState(uint64_t time, uint64_t stateTime) const;
		void stateEnter(ROCKET_STATE state, uint64_t time);
		void stateExit(ROCKET_STATE state, uint64_t time);
		void stateDo(ROCKET_STATE state, uint64_t time, uint64_t stateTime);

		void getRocketState(uint8_t *data, uint8_t &n);
		void setRocketState(uint8_t *data, uint8_t &n);
		void sendRemoteCommand(DeviceIds device_id, ROCKET_CMDs command);
		double getSensorReading(const ADCChannel &sensor_channel) const;

		void beepForAbortState(uint64_t current_time);

		const ADCChannel &fuelPressureChannel;
		const ADCChannel &oxPressureChannel;
		const ADCChannel &chamberPressureChannel;
		ServoChannel &fuelServoChannel;
		ServoChannel &oxServoChannel;
		PIControlChannel &piControlChannel;
		PyroChannel &internalIgniter1Channel;
		PyroChannel &internalIgniter2Channel;
		PyroChannel &ventValveChannel;

		Speaker &speaker;

		ROCKET_STATE state;
		ROCKET_STATE stateOverride;

		Can &can;

	    double sensor_slope = 0.01888275146;
	    double sensor_offset = -15;
		double chamberPressureMin = 12;
		double fuelPressureMin = 0;
		double oxPressureMin = 0;
		double tankPressureMax = 0;

		uint16_t chamberPressureLowCounter = 0;
		uint16_t chamberPressureGoodCounter = 0;
		uint16_t autoCheckBadCounter = 0;
		uint16_t holdDownTimeout = 3000;
		uint64_t timeLastSample = 0;
		uint64_t timeLastTransition = 0;
		uint64_t timeSinceBothMainValvesOpen = 0;

		// Whether we are currently emitting sound due to us being in the abort state.
		bool isBeepForAbortStateOn = false;
		// When we last changed whether we are beeping or not beeping while in the abort state.
		uint64_t beepForAbortStateOnOffChangedAt = 0;

		bool gse_connection_abort_enabled = false;
		uint64_t timeLastGSEConnectionMessage = 0;


};

#endif /*ROCKETCHANNEL_H*/
