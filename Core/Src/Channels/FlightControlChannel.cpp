#include <Channels/FlightControlChannel.h>
#include "STRHAL_GPIO.h"
#include "NodeInfos.h"

FlightControlChannel::FlightControlChannel(
		uint8_t id, BaroChannel &baroChannel, IMUChannel &xAccelChannel, IMUChannel &yAccelChannel, IMUChannel &zAccelChannel,
		IMUChannel &xGyroChannel, IMUChannel &yGyroChannel, IMUChannel &zGyroChannel,
		DigitalOutChannel &out0Channel, DigitalOutChannel &out1Channel,
		DigitalOutChannel &out2Channel, DigitalOutChannel &out3Channel,
		BaroChannel &verticalSpeed, BaroChannel &baroAltitude,
		//Speaker speaker,
		uint32_t refreshDivider
) :
	AbstractChannel(CHANNEL_TYPE_FLIGHT, id, refreshDivider),
	baroChannel(baroChannel), xAccelChannel(xAccelChannel), yAccelChannel(yAccelChannel), zAccelChannel(zAccelChannel),
	xGyroChannel(xGyroChannel), yGyroChannel(yGyroChannel), zGyroChannel(zGyroChannel),
	out0Channel(out0Channel), out1Channel(out1Channel),
	out2Channel(out2Channel), out3Channel(out3Channel),
	verticalSpeed(verticalSpeed), baroAltitude(baroAltitude),
	//speaker(speaker),
	state(INIT), stateOverride(UNCHANGED),
	can(Can::instance(0))
{
}

int FlightControlChannel::init() {
	flag = true;
	return 0;
}

int FlightControlChannel::reset() {
	state = INIT;
	stateOverride = UNCHANGED;
	return 0;
}

int FlightControlChannel::exec() {
	uint64_t time = STRHAL_Systick_GetTick();
	if ((time - timeLastSample) < EXEC_SAMPLE_TICKS) {
		return 0;
	}
	timeLastSample = time;

	uint64_t stateTime = time - timeLastTransition;

	// An external state override always takes precedence over any internal state transitions.
	// If there is no external override, the next state is computed internally via nextState.
	FLIGHT_STATE newState;
	if (stateOverride != UNCHANGED) {
		newState = stateOverride;
		stateOverride = UNCHANGED;
	} else {
		newState = nextState(time, stateTime);
	}

	// In case of a state transition, the previous state's exit action is performed, followed
	// by the new state's enter action.
	if (newState != UNCHANGED) {
		stateExit(state, time);
		timeLastTransition = time;
		stateEnter(newState, time);
		stateTime = time - timeLastTransition;
		state = newState;
	}

	// In any case, the do action of the current state after any possible state transitions
	// is performed.
	stateDo(state, time, stateTime);

	return 0;
}

FLIGHT_STATE FlightControlChannel::nextState(uint64_t time, uint64_t stateTime) const {

	switch (state) {
		case INIT:
			return PAD;
		case PAD:
			if (flag){			// Either measure or get command from LLServer or from main ECU
				return BOOST;
			}
			return UNCHANGED;
		case BOOST:
			if (stateTime > BOOST_DURATION) {			// Timeout based
				return COAST;
			}
			return UNCHANGED;
		case COAST:
			if (verticalSpeed.getMeasurement() > 30) {			// Measure Apogee // TODO: USE KALMAN FILTER
				return DESCEND_DROGUE;
			}
			return UNCHANGED;
		case DESCEND_DROGUE:
			if (baroAltitude.getMeasurement() > MAIN_DEPLOYMENT_ALTITUDE) {			// Measure altitude
				return DESCEND_MAIN;
			}
			return UNCHANGED;
		case DESCEND_MAIN:
			if (verticalSpeed.getMeasurement() < LANDING_SPEED) {			// Measure speed/altitude
				return LANDED;
			}
			if (verticalSpeed.getMeasurement() > NO_MAIN_THRESHOLD) {			// Measure speed/altitude after timeout
				return DESCEND_NO_MAIN;
			}
			return UNCHANGED;
		case DESCEND_NO_MAIN:
			if (verticalSpeed.getMeasurement() < LANDING_SPEED) {			// Measure speed/altitude
				return LANDED;
			}
			return UNCHANGED;
		case LANDED:
			if (stateTime > 2000) {			// Wait for command from LLServer to reset or timeout based
				return PAD;
			}
			return UNCHANGED;
		default:
			return UNCHANGED;
	}
}

void FlightControlChannel::stateEnter(FLIGHT_STATE state, uint64_t time) {

	// LEDs for visible status
	STRHAL_GPIO_t led1 = { GPIOC, 8, STRHAL_GPIO_TYPE_OPP };
	STRHAL_GPIO_t led2 = { GPIOC, 9, STRHAL_GPIO_TYPE_OPP };


	switch (state) {
		case INIT:
			STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_L);
			STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
			break;
		case PAD:
			STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
			STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
			break;
		case BOOST:
			STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_L);
			STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
			break;
		case COAST:
			STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
			STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
			break;
		case DESCEND_DROGUE:
			// FIRE DROGUE PYRO CHARGES
			STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_L);
			STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
			break;
		case DESCEND_MAIN:
			// FIRE MAIN PYRO CHARGES
			STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
			STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
			break;
		case LANDED:
			// Disable cameras
			STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_L);
			STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
			break;
		default: break;
	}
}
void FlightControlChannel::stateExit(FLIGHT_STATE state, uint64_t time) {
	switch (state) {
		case INIT:
			break;
		case PAD:
			break;
		case BOOST:
			break;
		case COAST:
			break;
		case DESCEND_DROGUE:
			break;
		case DESCEND_MAIN:
			break;
		case LANDED:
			flag = false;
			break;
		default: break;
	}
}
void FlightControlChannel::stateDo(FLIGHT_STATE state, uint64_t time, uint64_t stateTime) {
	switch (state) {
		case INIT:
			//STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
			//STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
			break;
		case PAD:
			break;
		case BOOST:
			break;
		case COAST:
			break;
		case DESCEND_DROGUE:
			// Wait a few seconds, then disable drogue pyro charges
			break;
		case DESCEND_MAIN:
			// Wait a few seconds, then disable main pyro charges
			break;
		case LANDED:
			break;
		default: break;
	}
}

int FlightControlChannel::processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) {
	switch (commandId) {
	case FLIGHT_REQ_INTERNAL_CONTROL:
		if (state == PAD) {
		}
		return 0;
	case FLIGHT_REQ_SET_FLIGHT_STATE:
		setFlightState(returnData, n);
		return 0;
	case FLIGHT_REQ_GET_FLIGHT_STATE:
		getFlightState(returnData, n);
		return 0;
	case FLIGHT_REQ_ABORT:
		//stateOverride = ABORT;
		return 0;
	case FLIGHT_REQ_END_OF_FLIGHT:
		return 0;
	case FLIGHT_REQ_AUTO_CHECK:
		return 0;
	default:
		return AbstractChannel::processMessage(commandId, returnData, n);
	}
}

int FlightControlChannel::getSensorData(uint8_t *data, uint8_t &n) {


	///////////////////////////////////
	//                               //
	// TODO: Implement Kalman Filter //
	//                               //
	///////////////////////////////////


	uint16_t *out = (uint16_t*) (data + n);
	*out = state;

	n += FLIGHT_DATA_N_BYTES;
	return 0;
}

int FlightControlChannel::setVariable(uint8_t variableId, int32_t data) {
	switch (variableId) {
	case FLIGHT_STATE_REFRESH_DIVIDER:
		refreshDivider = data;
		refreshCounter = 0;
		return 0;
	case FLIGHT_SENSOR_SLOPE:
		sensor_slope = (double) data / 1000.0;
		return 0;
	case FLIGHT_SENSOR_OFFSET:
		sensor_offset = (double) data / 1000.0;
		return 0;
	default:
		return -1;
	}
}

int FlightControlChannel::getVariable(uint8_t variableId, int32_t &data) const {
	switch (variableId) {
	case FLIGHT_STATE_REFRESH_DIVIDER:
		data = (int32_t) refreshDivider;
		return 0;
	case FLIGHT_SENSOR_SLOPE:
		data = (int32_t) (sensor_slope * 1000);
		return 0;
	case FLIGHT_SENSOR_OFFSET:
		data = (int32_t) (sensor_offset * 1000);
		return 0;
	default:
		return -1;
	}
}

void FlightControlChannel::getFlightState(uint8_t *data, uint8_t &n) {
	FlightStateResMsg_t *flightStateResponseMsg = (FlightStateResMsg_t*) data;
	flightStateResponseMsg->state = state;
	flightStateResponseMsg->status = FLIGHT_WRITABLE;
	n += sizeof(FlightStateResMsg_t);
}

void FlightControlChannel::setFlightState(uint8_t *data, uint8_t &n)
{
	const FlightStateReqMsg_t *flightStateRequestMsg = (FlightStateReqMsg_t*) data;
	FlightStateResMsg_t *flightStateResponseMsg = (FlightStateResMsg_t*) data;
	const FLIGHT_STATE requestedState = static_cast<FLIGHT_STATE>(flightStateRequestMsg->state);

	/*
	if ((state == RS_HOLDDOWN && requestedState == RS_POWERED_ASCENT)
	||  (state == RS_ABORT_IGNITION_TIMEOUT && requestedState == RS_PAD_IDLE)
	||  (state == RS_ABORT_HOLDDOWN && requestedState == RS_PAD_IDLE)
	||  (state == RS_ABORT && requestedState == RS_PAD_IDLE))
	{
		stateOverride = requestedState;
		rocketStateResponseMsg->state = state;
		rocketStateResponseMsg->status = SUCCESS;
	} else {
		rocketStateResponseMsg->state = state;
		rocketStateResponseMsg->status = FAILURE_WRITE_PROTECTED;
	}*/

	//flightStateResponseMsg->state = state;
	//flightStateResponseMsg->status = FLIGHT_FAILURE_WRITE_PROTECTED;


	stateOverride = requestedState;
	flightStateResponseMsg->state = state;
	flightStateResponseMsg->status = SUCCESS;

	n += sizeof(FlightStateResMsg_t);
}
