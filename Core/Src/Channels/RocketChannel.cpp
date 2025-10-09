#include <Channels/RocketChannel.h>
#include "STRHAL_GPIO.h"
#include "NodeInfos.h"

RocketChannel::RocketChannel(
	uint8_t id, const ADCChannel &fuelPressureChannel, const ADCChannel &oxPressureChannel,
	const ADCChannel &chamberPressureChannel, ServoChannel &fuelServoChannel,
	ServoChannel &oxServoChannel, PIControlChannel &piControlChannel, PyroChannel &internalIgniter1Channel,
	PyroChannel &internalIgniter2Channel, PyroChannel &ventValveChannel, Speaker &speaker, uint32_t refreshDivider
) :
	AbstractChannel(CHANNEL_TYPE_ROCKET, id, refreshDivider),
	fuelPressureChannel(fuelPressureChannel), oxPressureChannel(oxPressureChannel),
	chamberPressureChannel(chamberPressureChannel), fuelServoChannel(fuelServoChannel),
	oxServoChannel(oxServoChannel), piControlChannel(piControlChannel),
	internalIgniter1Channel(internalIgniter1Channel), internalIgniter2Channel(internalIgniter2Channel),
	ventValveChannel(ventValveChannel), speaker(speaker), state(RS_INIT), stateOverride(RS_UNCHANGED),
	can(Can::instance(0))
{
}

int RocketChannel::init() {
	return 0;
}

int RocketChannel::reset() {
	state = RS_INIT;
	stateOverride = RS_UNCHANGED;
	timeLastTransition = 0;
	sensor_slope = 0.01888275146;
	sensor_offset = -15;
	chamberPressureMin = 0;
	chamberPressureLowCounter = 0;
	chamberPressureGoodCounter = 0;
	autoCheckBadCounter = 0;
	return 0;
}

int RocketChannel::exec() {
	uint64_t time = STRHAL_Systick_GetTick();
	if ((time - timeLastSample) < EXEC_SAMPLE_TICKS) {
		return 0;
	}
	timeLastSample = time;

	uint64_t stateTime = time - timeLastTransition;

	// An external state override always takes precedence over any internal state transitions.
	// If there is no external override, the next state is computed internally via nextState.
	ROCKET_STATE newState;
	if (stateOverride != RS_UNCHANGED) {
		newState = stateOverride;
		stateOverride = RS_UNCHANGED;
	} else {
		newState = nextState(time, stateTime);
	}

	// In case of a state transition, the previous state's exit action is performed, followed
	// by the new state's enter action.
	if (newState != RS_UNCHANGED) {
		stateExit(state, time);
		timeLastTransition = time;
		stateEnter(newState, time);
		stateTime = time - timeLastTransition;
		state = newState;
	}

	// In any case, the do action of the current state after any possible state transitions
	// is performed.
	stateDo(state, time, stateTime);

	// Beep if we are in the abort state
	beepForAbortState(time);

	return 0;
}

#if defined(IS_MAIN_ECU)
ROCKET_STATE RocketChannel::nextState(uint64_t time, uint64_t stateTime) const {
	switch (state) {
	case RS_INIT:
		return RS_PAD_IDLE;
	case RS_PAD_IDLE:
		// Can only exit through CAN command
		return RS_UNCHANGED;
	case RS_AUTOCHECK:
		if (autoCheckBadCounter > AUTO_CHECK_BAD_COUNT_MAX) {
			return RS_ABORT;
		}
		if (stateTime > 2000) {
			return RS_PAD_IDLE;
		}
		return RS_UNCHANGED;
	case RS_ABORT:
		// Can only exit through CAN command
		return RS_UNCHANGED;

	case RS_IGNITION_INIT:
		if (stateTime > IGNITION_DELAY) {
			return RS_IGNITION_IGNITE1;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_IGNITE1:
		if (stateTime > 500) {
			return RS_IGNITION_IGNITE2;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_IGNITE2:
		if (stateTime > 2200) {
			return RS_ABORT_IGNITION_TIMEOUT;
		}
		if (stateTime > 2000) {
			return RS_IGNITION_OX_PRESSURIZE;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_OX_PRESSURIZE:
		if (stateTime > 500) {
			return RS_ABORT_IGNITION_TIMEOUT;
		}
		if (stateTime > 10) {
			return RS_IGNITION_OX_OPEN;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_OX_OPEN:
		if (stateTime > 700) {
			return RS_ABORT_IGNITION_TIMEOUT;
		}
		if (stateTime > 500) {
			return RS_IGNITION_FUEL_OPEN;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_FUEL_OPEN:
		if (stateTime > 700) {
			return RS_ABORT_IGNITION_TIMEOUT;
		}
		if (stateTime > 170) {
			return RS_IGNITION_FUEL_PRESSURIZE;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_FUEL_PRESSURIZE:
		if (stateTime > 500) {
			return RS_ABORT_IGNITION_TIMEOUT;
		}
		if (stateTime > 330) {
			return RS_IGNITION_MAIN_OPEN;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_MAIN_OPEN:
		if (stateTime > HOLDDOWN_DELAY) {
			return RS_IGNITION_IGNITER_OFF;
		}
		return RS_UNCHANGED;
	case RS_IGNITION_IGNITER_OFF:
		return RS_HOLDDOWN;

	case RS_HOLDDOWN:
		if ((time - timeSinceBothMainValvesOpen < holdDownTimeout) || (holdDownTimeout == 0)) {
			if (chamberPressureGoodCounter > CHAMBER_PRESSURE_GOOD_COUNT_MIN) {
				return RS_POWERED_ASCENT;
			}
		} else {
			return RS_ABORT_HOLDDOWN;
		}
		return RS_UNCHANGED;

	case RS_POWERED_ASCENT:
		if (stateTime > FLIGHT_BURN_TIME) {
			// motor burnout, close valves, IMPORTANT!: total burn time before shutoff is powered + unpowered ascent
			return RS_UNPOWERED_ASCENT;
		}
		return RS_UNCHANGED;

	case RS_UNPOWERED_ASCENT:
		if (stateTime > 15000) {
			return RS_DEPRESSURIZE;
		}
		return RS_UNCHANGED;

	case RS_DEPRESSURIZE:
		if (getSensorReading(oxPressureChannel) < 1.5 && getSensorReading(fuelPressureChannel) < 1.5) {
			return RS_PAD_IDLE;
		}
		return RS_UNCHANGED;

	default:
		return RS_UNCHANGED;
	}
}

void RocketChannel::stateEnter(ROCKET_STATE state, uint64_t time) {
	STRHAL_GPIO_t led1 = { GPIOC, 8, STRHAL_GPIO_TYPE_OPP };
	STRHAL_GPIO_t led2 = { GPIOC, 9, STRHAL_GPIO_TYPE_OPP };

	switch (state) {
	case RS_PAD_IDLE: {
		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
	} break;
	case RS_ABORT: {
		fuelServoChannel.setTargetPos(0);
		oxServoChannel.setTargetPos(0);
		sendRemoteCommand(DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		sendRemoteCommand(DEVICE_ID_OX_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		(void) internalIgniter1Channel.setState(0);
		(void) internalIgniter2Channel.setState(0);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_2, DIGITAL_OUT_STATE, 0);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_1, DIGITAL_OUT_STATE, 0);
	} break;
	case RS_ABORT_IGNITION_TIMEOUT: {
		fuelServoChannel.setTargetPos(0);
		oxServoChannel.setTargetPos(0);
		sendRemoteCommand(DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		sendRemoteCommand(DEVICE_ID_OX_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		(void) internalIgniter1Channel.setState(0);
		(void) internalIgniter2Channel.setState(0);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_2, DIGITAL_OUT_STATE, 0);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_1, DIGITAL_OUT_STATE, 0);
	} break;
	case RS_ABORT_HOLDDOWN: {
		fuelServoChannel.setTargetPos(0);
		oxServoChannel.setTargetPos(0);
		sendRemoteCommand(DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		sendRemoteCommand(DEVICE_ID_OX_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		(void) internalIgniter1Channel.setState(0);
		(void) internalIgniter2Channel.setState(0);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_2, DIGITAL_OUT_STATE, 0);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_1, DIGITAL_OUT_STATE, 0);
	} break;

	case RS_IGNITION_INIT: {
		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
		fuelServoChannel.setTargetPos(0);
		oxServoChannel.setTargetPos(0);
		(void) internalIgniter1Channel.setState(0);
		(void) internalIgniter2Channel.setState(0);
		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_2, DIGITAL_OUT_STATE, 1);
		can.SetRemoteVariable(DEVICE_ID_RCU_CAM_1, DIGITAL_OUT_STATE, 1);
		can.SetRemoteVariable(DEVICE_ID_RCU_GENERIC_CHANNEL, GENERIC_LORA_ENABLED, 300);

		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 1);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 1);
	} break;
	case RS_IGNITION_IGNITE1: {
		(void) internalIgniter1Channel.setState(65000);
	} break;
	case RS_IGNITION_IGNITE2: {
		(void) internalIgniter2Channel.setState(65000);
	} break;
	case RS_IGNITION_OX_PRESSURIZE: {
		sendRemoteCommand(DEVICE_ID_OX_ECU_ROCKET_CHANNEL, ROCKET_REQ_INTERNAL_CONTROL);
	} break;
	case RS_IGNITION_OX_OPEN: {
		oxServoChannel.setTargetPos(32768); // 50%
	} break;
	case RS_IGNITION_FUEL_OPEN: {
		fuelServoChannel.setTargetPos(32768); // 50%
	} break;
	case RS_IGNITION_FUEL_PRESSURIZE: {
		sendRemoteCommand(DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL, ROCKET_REQ_INTERNAL_CONTROL);
	} break;
	case RS_IGNITION_MAIN_OPEN: {
		oxServoChannel.setTargetPos(65535);
		fuelServoChannel.setTargetPos(65535);
	} break;
	case RS_IGNITION_IGNITER_OFF: {
		(void) internalIgniter1Channel.setState(0);
		(void) internalIgniter2Channel.setState(0);
		timeSinceBothMainValvesOpen = time;
	} break;
	case RS_POWERED_ASCENT: {
		can.SetRemoteVariable(DEVICE_ID_GSE_PNEU_1_HOLDDOWN, DIGITAL_OUT_STATE, 1);
	} break;
	case RS_UNPOWERED_ASCENT: {
		oxServoChannel.setTargetPos(0);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
	} break;

	case RS_DEPRESSURIZE: {
		oxServoChannel.setTargetPos(65535);
	} break;

	default: break;
	}
}
void RocketChannel::stateExit(ROCKET_STATE state, uint64_t time) {
	switch (state) {
	case RS_DEPRESSURIZE: {
		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_PRESSURANT_VALVE, SERVO_TARGET_POSITION, 65535);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURANT_VALVE, SERVO_TARGET_POSITION, 65535);
		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 0);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 0);
	} break;
	default: break;
	}
}
void RocketChannel::stateDo(ROCKET_STATE state, uint64_t time, uint64_t stateTime) {
	switch (state) {
	case RS_AUTOCHECK: {
		// TODO check Holddown
		if (internalIgniter1Channel.getContinuity() == 1 || internalIgniter2Channel.getContinuity() == 1 || //no continuity
				oxPressureChannel.getMeasurement() < oxPressureMin || fuelPressureChannel.getMeasurement() < fuelPressureMin)
		{
			autoCheckBadCounter++;
		} else {
			autoCheckBadCounter = 0;
		}
	} break;
	case RS_HOLDDOWN: {
		if (getSensorReading(chamberPressureChannel) < chamberPressureMin) {
			chamberPressureGoodCounter -= (chamberPressureGoodCounter > CHAMBER_PRESSURE_LOW_PENALTY) ? CHAMBER_PRESSURE_LOW_PENALTY : 0;
			chamberPressureLowCounter++;
		} else {
			chamberPressureLowCounter = 0;
			chamberPressureGoodCounter++;
		}

		if (chamberPressureLowCounter > CHAMBER_PRESSURE_LOW_COUNT_MAX) {
			chamberPressureGoodCounter = 0;
		}
	} break;
	default: break;
	}
}
#elif defined(IS_NOT_MAIN_ECU)
ROCKET_STATE RocketChannel::nextState(uint64_t time, uint64_t stateTime) const {
	switch (state) {
	case RS_INIT:
		return RS_PAD_IDLE;
	case RS_PRESSURIZE:
		if (stateTime > 55000) {
			return RS_DEPRESSURIZE;
		}
		return RS_UNCHANGED;
	case RS_DEPRESSURIZE:
		if (stateTime > 500) {
			return RS_PAD_IDLE;
		}
		return RS_UNCHANGED;
	case RS_ABORT:
		if (stateTime > 500) {
			return RS_PAD_IDLE;
		}
		return RS_UNCHANGED;
	default:
		return RS_UNCHANGED;
	}
}

void RocketChannel::stateEnter(ROCKET_STATE state, uint64_t time) {
	STRHAL_GPIO_t led1 = { GPIOC, 8, STRHAL_GPIO_TYPE_OPP };
	STRHAL_GPIO_t led2 = { GPIOC, 9, STRHAL_GPIO_TYPE_OPP };

	switch (state) {
	case RS_PAD_IDLE: {
		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
	} break;
	case RS_PRESSURIZE: {
		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_L);
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
		ventValveChannel.setState(1);
		piControlChannel.setEnabled(1);
	} break;
	case RS_DEPRESSURIZE: {
		piControlChannel.setEnabled(0);
		ventValveChannel.setState(0);
	} break;
	case RS_ABORT: {
		piControlChannel.setEnabled(0);
		ventValveChannel.setState(0);
		internalIgniter1Channel.setState(0);
		internalIgniter2Channel.setState(0);
	} break;
	default: break;
	}
}
void RocketChannel::stateExit(ROCKET_STATE state, uint64_t time) {
}
void RocketChannel::stateDo(ROCKET_STATE state, uint64_t time, uint64_t stateTime) {
}
#else
#error config error
#endif

int RocketChannel::processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) {
	switch (commandId) {
	case ROCKET_REQ_INTERNAL_CONTROL:
		if (state == RS_PAD_IDLE) {
#if defined(IS_MAIN_ECU)
			stateOverride = RS_IGNITION_INIT;
#elif defined(IS_NOT_MAIN_ECU)
			stateOverride = RS_PRESSURIZE;
#else
	#error config error
#endif
		}
		return 0;
	case ROCKET_REQ_SET_ROCKET_STATE:
		setRocketState(returnData, n);
		return 0;
	case ROCKET_REQ_GET_ROCKET_STATE:
		getRocketState(returnData, n);
		return 0;
	case ROCKET_REQ_ABORT:
		stateOverride = RS_ABORT;
		return 0;
	case ROCKET_REQ_END_OF_FLIGHT: // TODO: Reachable?
#if defined(IS_MAIN_ECU)
		if (state == RS_UNPOWERED_ASCENT) {
			stateOverride = RS_DEPRESSURIZE;
		}
#elif !defined(IS_NOT_MAIN_ECU)
	#error config error
#endif
		return 0;
	case ROCKET_REQ_AUTO_CHECK:
#if defined(IS_MAIN_ECU) // TODO: Correct?
		if (state == RS_PAD_IDLE) {
			stateOverride = RS_AUTOCHECK;
		}
#elif !defined(IS_NOT_MAIN_ECU)
	#error config error
#endif
		return 0;
	default:
		return AbstractChannel::processMessage(commandId, returnData, n);
	}
}

int RocketChannel::getSensorData(uint8_t *data, uint8_t &n) {
	uint16_t *out = (uint16_t*) (data + n);
	*out = state;

	n += ROCKET_DATA_N_BYTES;
	return 0;
}

int RocketChannel::setVariable(uint8_t variableId, int32_t data) {
	switch (variableId) {
	case ROCKET_STATE_REFRESH_DIVIDER:
		refreshDivider = data;
		refreshCounter = 0;
		return 0;
	case ROCKET_SENSOR_SLOPE:
		sensor_slope = (double) data / 1000.0;
		return 0;
	case ROCKET_SENSOR_OFFSET:
		sensor_offset = (double) data / 1000.0;
		return 0;
	case ROCKET_MINIMUM_CHAMBER_PRESSURE:
		chamberPressureMin = (double) data / 1000.0;
		return 0;
	case ROCKET_MINIMUM_FUEL_PRESSURE:
		fuelPressureMin = (double) data / 1000.0;
		return 0;
	case ROCKET_MINIMUM_OX_PRESSURE:
		oxPressureMin = (double) data / 1000.0;
		return 0;
	case ROCKET_HOLDDOWN_TIMEOUT:
		holdDownTimeout = data;
		return 0;
	default:
		return -1;
	}
}

int RocketChannel::getVariable(uint8_t variableId, int32_t &data) const {
	switch (variableId) {
	case ROCKET_STATE_REFRESH_DIVIDER:
		data = (int32_t) refreshDivider;
		return 0;
	case ROCKET_SENSOR_SLOPE:
		data = (int32_t) (sensor_slope * 1000);
		return 0;
	case ROCKET_SENSOR_OFFSET:
		data = (int32_t) (sensor_offset * 1000);
		return 0;
	case ROCKET_MINIMUM_CHAMBER_PRESSURE:
		data = (int32_t) (chamberPressureMin * 1000);
		return 0;
	case ROCKET_MINIMUM_FUEL_PRESSURE:
		data = (int32_t) (fuelPressureMin * 1000);
		return 0;
	case ROCKET_MINIMUM_OX_PRESSURE:
		data = (int32_t) (oxPressureMin * 1000);
		return 0;
	case ROCKET_HOLDDOWN_TIMEOUT:
		data = (int32_t) holdDownTimeout;
		return 0;
	default:
		return -1;
	}
}

void RocketChannel::getRocketState(uint8_t *data, uint8_t &n) {
	RocketStateResMsg_t *rocketStateResponseMsg = (RocketStateResMsg_t*) data;
	rocketStateResponseMsg->state = state;
	rocketStateResponseMsg->status = WRITABLE;
	n += sizeof(RocketStateResMsg_t);
}

void RocketChannel::setRocketState(uint8_t *data, uint8_t &n)
{
	const RocketStateReqMsg_t *rocketStateRequestMsg = (RocketStateReqMsg_t*) data;
	RocketStateResMsg_t *rocketStateResponseMsg = (RocketStateResMsg_t*) data;
	const ROCKET_STATE requestedState = static_cast<ROCKET_STATE>(rocketStateRequestMsg->state);

#if defined(IS_MAIN_ECU)
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
	}
#elif defined(IS_NOT_MAIN_ECU)
	rocketStateResponseMsg->state = state;
	rocketStateResponseMsg->status = FAILURE_WRITE_PROTECTED;
#else
	#error config error
#endif
	n += sizeof(RocketStateResMsg_t);
}


void RocketChannel::sendRemoteCommand(DeviceIds device_id, ROCKET_CMDs command) {
	uint8_t empty_buffer[10] =
	{ 0 };
	can.sendAsMaster(device_id, command, empty_buffer, sizeof(uint32_t));
}

double RocketChannel::getSensorReading(const ADCChannel &sensor_channel) const {
	uint16_t data = sensor_channel.getMeasurement();
	return ((double) data * sensor_slope + sensor_offset);
}

void RocketChannel::beepForAbortState(uint64_t current_time) {
	if (state == ROCKET_STATE::RS_ABORT) {
		// 500ms beeps, every 3s.
		uint64_t toggleDuration = isBeepForAbortStateOn ? 500 : 2500;
		if (current_time - beepForAbortStateOnOffChangedAt > toggleDuration) {
			beepForAbortStateOnOffChangedAt = current_time;
			isBeepForAbortStateOn = !isBeepForAbortStateOn;
			speaker.enable(isBeepForAbortStateOn);
		}
	} else {
		isBeepForAbortStateOn = false;
		beepForAbortStateOnOffChangedAt = 0;
		speaker.enable(false);
	}
}
