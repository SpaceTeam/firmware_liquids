#include <Channels/RocketChannel.h>

RocketChannel::RocketChannel(uint8_t id, const ADCChannel &oxPressureChannel, const ADCChannel &fuelPressureChannel, const ADCChannel &chamberPressureChannel, ServoChannel &oxServoChannel, ServoChannel &fuelServoChannel, PyroChannel &igniter0Channel, PyroChannel &igniter1Channel, uint32_t refreshDivider)
	: AbstractChannel(CHANNEL_TYPE_ROCKET, id, refreshDivider),
	  oxPressureChannel(oxPressureChannel),
	  fuelPressureChannel(fuelPressureChannel),
	  chamberPressureChannel(chamberPressureChannel),
	  oxServoChannel(oxServoChannel),
	  fuelServoChannel(fuelServoChannel),
	  igniter0Channel(igniter0Channel),
	  igniter1Channel(igniter1Channel),
	  state(PAD_IDLE),
	  ignitionState(IgnitionSequence::INIT),
	  chamberPressureMin(0),
	  chamberPressureLowCounter(0),
	  chamberPressureGoodCounter(0),
	  fuelPressureMin(0),
	  oxPressureMin(0),
	  holdDownTimeout(0) {
}

int RocketChannel::init() {
	cancom = CANCOM::instance(nullptr); // works because the generic channel (e.g. ECU) has already initialized CANCOM
	return 0;
}

int RocketChannel::exec() {
	char buf[64];

	uint64_t time = STRHAL_Systick_GetTick();
	if((time - timeLastSample) < EXEC_SAMPLE_TICKS)
		return 0;

	timeLastSample = time;

	// Current State Logic - executes state logic, also returns new state if transition conditions are met
	internalNextState = currentStateLogic(time);

	ROCKET_STATE nextState = state;

	if(externalNextState != state) { // Prioritize external event - there has to be some kind of priority, because internal could be different to external -> external means CAN -> either Sequence or Abort
		nextState = externalNextState;
	} else if(internalNextState != state) {
		nextState = internalNextState;
	}

	// Next State Logic
	if(nextState != state) {
		sprintf(buf,"next state: %d",nextState);
		STRHAL_UART_Write(buf, strlen(buf));
		nextStateLogic(nextState, time);
	}

	return 0;
}

ROCKET_STATE RocketChannel::currentStateLogic(uint64_t time) {
	switch(state) {
		case PAD_IDLE:
			// wait until auto sequence command from CAN
			break;
		case AUTO_CHECK:
			return autoCheck(time);
		case IGNITION_SEQUENCE:
			return ignitionSequence(time);
		case HOLD_DOWN:
			return holddown(time);
		case POWERED_ASCENT:
			return poweredAscent(time);
		case UNPOWERED_ASCENT:
			// wait until end of flight command from PMU2
			break;
		case DEPRESS:
			return depress(time);
		case ABORT:
			return abort(time);
		default:
			break;
	}
	return state;
}

void RocketChannel::nextStateLogic(ROCKET_STATE nextState, uint64_t time) {
	timeLastTransition = time;
	this->state = nextState;
}

ROCKET_STATE RocketChannel::autoCheck(uint64_t time) {
	if(time - timeLastTransition > 2000) {
		return IGNITION_SEQUENCE;
	}
	// TODO check Holddown
	if(igniter0Channel.getContinuity() == 1) //no continuity
		return ABORT;

	if(igniter1Channel.getContinuity() == 1) //no continuity
		return ABORT;

	if(oxPressureChannel.getMeasurement() < oxPressureMin) //TODO set using CAN interface OR hardcode 35 bar
		return ABORT;

	if(fuelPressureChannel.getMeasurement() < fuelPressureMin) //TODO set using CAN interface OR hardcode 30 bar
		return ABORT;

	return AUTO_CHECK;
}

ROCKET_STATE RocketChannel::ignitionSequence(uint64_t time) {
	switch(ignitionState) {
		case IgnitionSequence::INIT: // Start of Ignition Sequence
			if(time - timeLastTransition >= 0) {
				fuelServoChannel.setTargetPos(0);
				oxServoChannel.setTargetPos(0);
				(void) igniter0Channel.setState(0);
				(void) igniter1Channel.setState(0);
				ignitionState = IgnitionSequence::IGNITION_ON;
			}
			break;
		case IgnitionSequence::IGNITION_ON: // T-3 - Ignition
			if(time - timeLastTransition > 7000) {
				(void) igniter0Channel.setState(65000);
				(void) igniter1Channel.setState(65000);
				ignitionState = IgnitionSequence::T_0;
			}
			break;
		case IgnitionSequence::T_0: // T - Valves to 0
			if(time - timeLastTransition > 10000) {
				fuelServoChannel.setTargetPos(0);
				oxServoChannel.setTargetPos(0);
				ignitionState = IgnitionSequence::VALVES_TO_20;
			}
			break;
		case IgnitionSequence::VALVES_TO_20: // T+0.5 - Valves to 20% open
			if(time - timeLastTransition > 10500) {
				fuelServoChannel.setTargetPos(15000);
				oxServoChannel.setTargetPos(15000);
				ignitionState = IgnitionSequence::VALVES_TO_40;
			}
			break;
		case IgnitionSequence::VALVES_TO_40: // T+1.2 - Valves to 40% open
			if(time - timeLastTransition > 11200) {
				fuelServoChannel.setTargetPos(25000);
				oxServoChannel.setTargetPos(25000);
				ignitionState = IgnitionSequence::IGNITION_OFF;
			}
			break;
		case IgnitionSequence::IGNITION_OFF: // T+1.5 - Ignition off
			if(time - timeLastTransition > 11500) {
				(void) igniter0Channel.setState(0);
				(void) igniter1Channel.setState(0);
				ignitionState = IgnitionSequence::VALVES_TO_100;
			}
			break;
		case IgnitionSequence::VALVES_TO_100: // T+1.7 - Valves to 100% open
			if(time - timeLastTransition > 11700) {
				fuelServoChannel.setTargetPos(65000);
				oxServoChannel.setTargetPos(65000);
				return HOLD_DOWN;
			}
			break;
	}

	return IGNITION_SEQUENCE;

}

ROCKET_STATE RocketChannel::holddown(uint64_t time) {
	if(holdDownTimeout > 0) {
		if(time - timeLastTransition > holdDownTimeout) { // release after x s to lessen apogee
			if(chamberPressureChannel.getMeasurement() < chamberPressureMin) {// if holddown timeout has passed, still check for chamber pressure
				chamberPressureGoodCounter = 0;
				chamberPressureLowCounter++;
			} else {
				chamberPressureLowCounter = 0;
				chamberPressureGoodCounter++;
			}

			// if either event (low or good pressure) occurs exclusively for a specified amount of times -> abort (low)/release(good)
			if(chamberPressureLowCounter > CHAMBER_PRESSURE_LOW_COUNT_MAX) {
				chamberPressureLowCounter = 0;
				return ABORT;
			}

			if(chamberPressureGoodCounter > CHAMBER_PRESSURE_GOOD_COUNT_MIN) {
				//TODO calibrate holddown servo
				if(cancom == nullptr)
					return ABORT;

				SetMsg_t setMsg =
				{ 0 };
				setMsg.variable_id = 1; // servo target position
				setMsg.value = 65000; // open servo
				cancom->sendAsMaster(9, 11, 4, (uint8_t *) &setMsg, 5); // send REQ_SET_VARIABLE (4) command to holddown servo (channelId 11) on oxcart node (nodeId 9)
				return POWERED_ASCENT;
			}
		}
	} else {
		if(chamberPressureChannel.getMeasurement() >= chamberPressureMin) {
			if(cancom == nullptr)
				return ABORT;

			SetMsg_t setMsg =
			{ 0 };
			setMsg.variable_id = 1; // servo target position
			setMsg.value = 65000; // open servo
			cancom->sendAsMaster(9, 11, 4, (uint8_t *) &setMsg, 5); // send REQ_SET_VARIABLE (4) command to holddown servo (channelId 11) on oxcart node (nodeId 9)
			return POWERED_ASCENT;
		}
	}
	return HOLD_DOWN;

}

ROCKET_STATE RocketChannel::poweredAscent(uint64_t time) {
	if(time - timeLastTransition > 6000) { // motor burnout, close valves, IMPORTANT!: total burn time before shutoff is powered + unpowered ascent
		fuelServoChannel.setTargetPos(0);
		oxServoChannel.setTargetPos(0);
		return UNPOWERED_ASCENT;
	}
	return POWERED_ASCENT;
}

ROCKET_STATE RocketChannel::depress(uint64_t time) {
	if(time - timeLastTransition > 6000) { // PMU2 sent end of flight, depress rocket after a short waiting period
		fuelServoChannel.setTargetPos(65000);
		oxServoChannel.setTargetPos(65000);
		// TODO add complete depress sequence and maybe check chamber pressure to eliminate point of failure from wrong end of flight command
		return UNPOWERED_ASCENT;
	}
	return DEPRESS;
}

ROCKET_STATE RocketChannel::abort(uint64_t time) {
	fuelServoChannel.setTargetPos(0);
	oxServoChannel.setTargetPos(0);
	return ABORT;
}

int RocketChannel::reset() {
	externalNextState = PAD_IDLE;
	ignitionState = IgnitionSequence::INIT;
	return 0;
}

int RocketChannel::processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) {
	switch(commandId) {
		case ROCKET_REQ_INTERNAL_CONTROL:
			if(state == PAD_IDLE) {
				externalNextState = AUTO_CHECK;
			}
			return 0;
		case ROCKET_REQ_ABORT:
			externalNextState = ABORT;
			return 0;
		case ROCKET_REQ_END_OF_FLIGHT:
			if(state == UNPOWERED_ASCENT) {
				externalNextState = DEPRESS;
			}
			return 0;
		case ROCKET_REQ_SET_ROCKET_STATE:
			setRocketState(returnData,n);
			return 0;
		case ROCKET_REQ_GET_ROCKET_STATE:
			getRocketState(returnData,n);
			return 0;
		default:
			return AbstractChannel::processMessage(commandId, returnData, n);
	}
}

int RocketChannel::getSensorData(uint8_t *data, uint8_t &n) {
	uint16_t *out = (uint16_t *) (data+n);
	*out = state;

	n += ROCKET_DATA_N_BYTES;
	return 0;
}

int RocketChannel::setVariable(uint8_t variableId, int32_t data) {
	switch(variableId) {
		case ROCKET_STATE_REFRESH_DIVIDER:
			refreshDivider = data;
			refreshCounter = 0;
			return 0;
		case ROCKET_MINIMUM_CHAMBER_PRESSURE:
			chamberPressureMin = data;
			return 0;
		case ROCKET_MINIMUM_FUEL_PRESSURE:
			fuelPressureMin = data;
			return 0;
		case ROCKET_MINIMUM_OX_PRESSURE:
			oxPressureMin = data;
			return 0;
		case ROCKET_HOLDDOWN_TIMEOUT:
			holdDownTimeout = data;
			return 0;
		default:
			return -1;
	}
}

int RocketChannel::getVariable(uint8_t variableId, int32_t &data) const {
	switch(variableId) {
		case ROCKET_STATE_REFRESH_DIVIDER:
			data = (int32_t) refreshDivider;
			return 0;
		case ROCKET_MINIMUM_CHAMBER_PRESSURE:
			data = (int32_t) chamberPressureMin;
			return 0;
		case ROCKET_MINIMUM_FUEL_PRESSURE:
			data = (int32_t) fuelPressureMin;
			return 0;
		case ROCKET_MINIMUM_OX_PRESSURE:
			data = (int32_t) oxPressureMin;
			return 0;
		case ROCKET_HOLDDOWN_TIMEOUT:
			data = (int32_t) holdDownTimeout;
			return 0;
		default:
			return -1;
	}
}

void RocketChannel::setRocketState(uint8_t *data, uint8_t &n) {
	RocketStateReqMsg_t * rocketStateRequestMsg;
	RocketStateResMsg_t * rocketStateResponseMsg;
	rocketStateRequestMsg = (RocketStateReqMsg_t *) data;
	externalNextState = static_cast<ROCKET_STATE>(rocketStateRequestMsg->state);

	rocketStateResponseMsg = (RocketStateResMsg_t *) data;
	rocketStateResponseMsg->state = state;
	rocketStateResponseMsg->status = SUCCESS;
	n+=sizeof(RocketStateResMsg_t);
}

void RocketChannel::getRocketState(uint8_t *data, uint8_t &n) {
	RocketStateResMsg_t * rocketStateResponseMsg;

	rocketStateResponseMsg = (RocketStateResMsg_t *) data;
	rocketStateResponseMsg->state = state;
	rocketStateResponseMsg->status = WRITABLE;
	n+=sizeof(RocketStateResMsg_t);
}
