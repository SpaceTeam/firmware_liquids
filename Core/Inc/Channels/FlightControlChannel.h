#ifndef FLIGHTCONTROLCHANNEL_H
#define FLIGHTCONTROLCHANNEL_H

#include "./Channels/AbstractChannel.h"
#include "./Channels/IMUChannel.h"
#include "./Channels/BaroChannel.h"
#include "./Channels/DigitalOutChannel.h"
#include "./Channels/ServoChannel.h"
#include "./Can.h"
#include "./Speaker.h"
#include <can_houbolt/channels/flight_control_channel_def.h>
#include <STRHAL.h>
#include <STRHAL_UART.h>
#include <cstring>
#include <cstdio>

class FlightControlChannel: public AbstractChannel
{
	public:
		FlightControlChannel(
			uint8_t id, BaroChannel &baroChannel, IMUChannel &xAccelChannel, IMUChannel &yAccelChannel, IMUChannel &zAccelChannel,
			IMUChannel &xGyroChannel, IMUChannel &yGyroChannel, IMUChannel &zGyroChannel,
			DigitalOutChannel &out0Channel, DigitalOutChannel &out1Channel,
			DigitalOutChannel &out2Channel, DigitalOutChannel &out3Channel,
			BaroChannel &vertcalSpeed, BaroChannel &baroAltitude,
			//Speaker speaker,
			uint32_t refreshDivider
		);
		FlightControlChannel(const FlightControlChannel &other) = delete;
		FlightControlChannel& operator=(const FlightControlChannel &other) = delete;
		FlightControlChannel(const FlightControlChannel &&other) = delete;
		FlightControlChannel& operator=(const FlightControlChannel &&other) = delete;

		int init() override;
		int reset() override;
		int exec() override;
		int getSensorData(uint8_t *data, uint8_t &n) override;
		int processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) override;
		void setFlightState(uint8_t *data, uint8_t &n);
		void getFlightState(uint8_t *data, uint8_t &n);

	protected:

		int setVariable(uint8_t variableId, int32_t data) override;
		int getVariable(uint8_t variableId, int32_t &data) const override;

	private:
		static constexpr uint16_t EXEC_SAMPLE_TICKS = 1;

		FLIGHT_STATE nextState(uint64_t time, uint64_t stateTime) const;
		void stateEnter(FLIGHT_STATE state, uint64_t time);
		void stateExit(FLIGHT_STATE state, uint64_t time);
		void stateDo(FLIGHT_STATE state, uint64_t time, uint64_t stateTime);



		BaroChannel &baroChannel;
		IMUChannel &xAccelChannel;
		IMUChannel &yAccelChannel;
		IMUChannel &zAccelChannel;
		IMUChannel &xGyroChannel;
		IMUChannel &yGyroChannel;
		IMUChannel &zGyroChannel;
		DigitalOutChannel &out0Channel;
		DigitalOutChannel &out1Channel;
		DigitalOutChannel &out2Channel;
		DigitalOutChannel &out3Channel;
		BaroChannel &verticalSpeed, &baroAltitude;
		//Speaker speaker = Speaker(STRHAL_TIM_TIM2, STRHAL_TIM_TIM2_CH3_PB10);

		FLIGHT_STATE state;
		FLIGHT_STATE stateOverride;

		Can &can;

		bool flag;

	    double sensor_slope = 0.01888275146;
	    double sensor_offset = -15;

	    uint16_t MAIN_DEPLOYMENT_ALTITUDE = 450;
	    uint16_t NO_MAIN_THRESHOLD = 45;
	    uint16_t BOOST_DURATION = 10000;
	    uint16_t LANDING_SPEED = 5;

		uint64_t timeLastSample = 0;
		uint64_t timeLastTransition = 0;

};

#endif /*ADCCHANNEL_H*/
