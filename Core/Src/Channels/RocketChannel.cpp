#include <Channels/RocketChannel.h>
#include "STRHAL_GPIO.h"
RocketChannel::RocketChannel(uint8_t id, const ADCChannel &fuelPressureChannel, const ADCChannel &oxPressureChannel, const ADCChannel &chamberPressureChannel,
		ServoChannel &fuelServoChannel, ServoChannel &oxServoChannel, PIControlChannel &piControlChannel, PyroChannel &internalIgniter1Channel, PyroChannel &internalIgniter2Channel, PyroChannel &ventValveChannel,
		uint8_t is_main_ecu, uint32_t refreshDivider) :
		AbstractChannel(CHANNEL_TYPE_ROCKET, id, refreshDivider), fuelPressureChannel(fuelPressureChannel), oxPressureChannel(oxPressureChannel), chamberPressureChannel(
				chamberPressureChannel), fuelServoChannel(fuelServoChannel), oxServoChannel(oxServoChannel), piControlChannel(piControlChannel), internalIgniter1Channel(
						internalIgniter1Channel), internalIgniter2Channel(internalIgniter2Channel), ventValveChannel(ventValveChannel), is_main_ecu(is_main_ecu), state(PAD_IDLE), ignitionState(IgnitionSequence::INIT), can(Can::instance(0))

{
}

int RocketChannel::init()
{
	return 0;
}

int RocketChannel::exec()
{
	uint64_t time = STRHAL_Systick_GetTick();
	if ((time - timeLastSample) < EXEC_SAMPLE_TICKS)
		return 0;

	timeLastSample = time;

	// Current State Logic - executes state logic, also returns new state if transition conditions are met
	internalNextState = currentStateLogic(time);

	ROCKET_STATE nextState = state;

	if (externalNextState != state)
	{ // Prioritize external event - there has to be some kind of priority, because internal could be different to external -> external means CAN -> either Sequence or Abort
		nextState = externalNextState;
	}
	else if (internalNextState != state)
	{
		externalNextState = internalNextState; // Incase an internal state change happens, the external state, which is from some previous change would block it, so it is updated here
		nextState = internalNextState;
	}

	// Next State Logic
	if (nextState != state)
	{
		nextStateLogic(nextState, time);
	}

	return 0;
}

ROCKET_STATE RocketChannel::currentStateLogic(uint64_t time)
{
	switch (state)
	{
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
		if (time - timeLastTransition > 45000)
		{
			return DEPRESS;
		}
		break;
	case DEPRESS:
		return depress(time);
	case ABORT:
		return abort(time);
	case PRESSURIZE_TANKS:
		return pressurize_tanks(time);
	default:
		break;
	}
	return state;
}

void RocketChannel::nextStateLogic(ROCKET_STATE nextState, uint64_t time)
{
	STRHAL_GPIO_t led1 =
	{ GPIOC, 8, STRHAL_GPIO_TYPE_OPP };
	STRHAL_GPIO_t led2 =
	{ GPIOC, 9, STRHAL_GPIO_TYPE_OPP };
	timeLastTransition = time;
	switch (nextState)
	{
	case PAD_IDLE:
		ignitionState = IgnitionSequence::INIT;
		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
		break;
	case AUTO_CHECK:
		break;
	case IGNITION_SEQUENCE:

		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);

		break;
	case HOLD_DOWN:
		break;
	case POWERED_ASCENT:
	{
		can.SetRemoteVariable(DEVICE_ID_GSE_PNEU_HOLDDOWN, DIGITAL_OUT_STATE, 1);
		break;
	}
	case UNPOWERED_ASCENT:
		piControlChannel.setEnabled(0);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
		break;
	case DEPRESS:
		oxServoChannel.setTargetPos(65535);


		//if (chamberPressureChannel.getMeasurement() > chamberPressureMin)
		//return; // do not set next state to DEPRESS if there is still combustion going on
		//can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 0);
		//can.SetRemoteVariable(DEVICE_ID_OX_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 0);

		//SetRemoteRocketState(DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		//SetRemoteRocketState(DEVICE_ID_OX_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
		break;
	case ABORT:
		if (is_main_ecu)
		{
			fuelServoChannel.setTargetPos(0);
			piControlChannel.setEnabled(0);
			SetRemoteRocketState(DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
			SetRemoteRocketState(DEVICE_ID_OX_ECU_ROCKET_CHANNEL, ROCKET_REQ_ABORT);
			(void) internalIgniter1Channel.setState(0);
			(void) internalIgniter2Channel.setState(0);
		}
		else
		{
			piControlChannel.setEnabled(1);
			ventValveChannel.setState(0);
		}
		break;
	case PRESSURIZE_TANKS:

		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_L);
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
		ventValveChannel.setState(1);
		piControlChannel.setEnabled(1);

		break;

	default:
		break;
	}
	state = nextState;
	return;
}

ROCKET_STATE RocketChannel::autoCheck(uint64_t time)
{
	if (time - timeLastTransition > 2000)
	{
		return PAD_IDLE;
	}
	// TODO check Holddown
	if (internalIgniter1Channel.getContinuity() == 1 || internalIgniter2Channel.getContinuity() == 1 || //no continuity
			oxPressureChannel.getMeasurement() < oxPressureMin || fuelPressureChannel.getMeasurement() < fuelPressureMin)
	{

		autoCheckBadCounter++;
	}
	else
	{
		autoCheckBadCounter = 0;
	}

	if (autoCheckBadCounter > AUTO_CHECK_BAD_COUNT_MAX)
		return ABORT;

	return AUTO_CHECK;
}

#define IGNITIONTIME(x) ( 10000 + x)

ROCKET_STATE RocketChannel::ignitionSequence(uint64_t time)
{
	switch (ignitionState)
	{
	case IgnitionSequence::INIT: // Start of Ignition Sequence
		if (time - timeLastTransition >= IGNITIONTIME(-10000))
		{
			fuelServoChannel.setTargetPos(0);
			oxServoChannel.setTargetPos(0);
			(void) internalIgniter1Channel.setState(0);
			(void) internalIgniter2Channel.setState(0);
			can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
			can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
			//can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_PRESSURE_CONTROLLER, PI_CONTROL_TARGET, 25000);
			//can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER, PI_CONTROL_TARGET, 25000);
			can.SetRemoteVariable(DEVICE_ID_RCU_CAM_2, DIGITAL_OUT_STATE, 1);
			can.SetRemoteVariable(DEVICE_ID_RCU_CAM_1, DIGITAL_OUT_STATE, 1);

			can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 1);
			can.SetRemoteVariable(DEVICE_ID_OX_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 1);

			ignitionState = IgnitionSequence::IGNITION_ON1;
		}
		break;
	case IgnitionSequence::IGNITION_ON1:
		if (time - timeLastTransition > IGNITIONTIME(0))
		{
			(void) internalIgniter1Channel.setState(65000);
			ignitionState = IgnitionSequence::IGNITION_ON2;
		}
		break;
	case IgnitionSequence::IGNITION_ON2:
		if (time - timeLastTransition > IGNITIONTIME(500))
		{
			(void) internalIgniter2Channel.setState(65000);
			ignitionState = IgnitionSequence::ENABLE_OX_PRESSURANT;
		}
		break;
	case IgnitionSequence::ENABLE_OX_PRESSURANT:
		if (time - timeLastTransition > IGNITIONTIME(1800))
		{
			SetRemoteRocketState(DEVICE_ID_OX_ECU_ROCKET_CHANNEL, ROCKET_REQ_INTERNAL_CONTROL);
			ignitionState = IgnitionSequence::OPEN_OX_MAIN;
		}
		break;
	case IgnitionSequence::OPEN_OX_MAIN:
		if (time - timeLastTransition > IGNITIONTIME(2000))
		{
			piControlChannel.setEnabled(1);
			ignitionState = IgnitionSequence::ENABLE_FUEL_PRESSURANT;
		}
		break;
	case IgnitionSequence::ENABLE_FUEL_PRESSURANT:
		if (time - timeLastTransition > IGNITIONTIME(2300))
		{
			SetRemoteRocketState(DEVICE_ID_FUEL_ECU_ROCKET_CHANNEL, ROCKET_REQ_INTERNAL_CONTROL);
			ignitionState = IgnitionSequence::OPEN_FUEL_MAIN;
		}
		break;
	case IgnitionSequence::OPEN_FUEL_MAIN:
		if (time - timeLastTransition > IGNITIONTIME(2500))
		{

			fuelServoChannel.setTargetPos(65535);
			timeSinceBothMainValvesOpen = time;
			ignitionState = IgnitionSequence::IGNITION_OFF;
		}
		break;
	case IgnitionSequence::IGNITION_OFF:
		if (time - timeLastTransition > IGNITIONTIME(2500 + HOLDDOWN_DELAY)) // IMPORTANT: DELAYS HOLDDOWN RELEASE START
		{
			(void) internalIgniter1Channel.setState(0);
			(void) internalIgniter2Channel.setState(0);
			return HOLD_DOWN;
		}
		break;
	}

	return IGNITION_SEQUENCE;

}

ROCKET_STATE RocketChannel::holddown(uint64_t time)
{
	if ((time - timeSinceBothMainValvesOpen < holdDownTimeout) || (holdDownTimeout == 0))
	{
		if ( GetSensorReading(chamberPressureChannel) < chamberPressureMin)
		{
			chamberPressureGoodCounter -= (chamberPressureGoodCounter > CHAMBER_PRESSURE_LOW_PENALTY) ? CHAMBER_PRESSURE_LOW_PENALTY : 0;
			chamberPressureLowCounter++;
		}
		else
		{
			chamberPressureLowCounter = 0;
			chamberPressureGoodCounter++;
		}

		if (chamberPressureLowCounter > CHAMBER_PRESSURE_LOW_COUNT_MAX)
		{
			chamberPressureGoodCounter = 0;
		}
		if (chamberPressureGoodCounter > CHAMBER_PRESSURE_GOOD_COUNT_MIN)
		{
			return POWERED_ASCENT;

		}
	}
	else
	{
		return ABORT;
	}
	return HOLD_DOWN;
}

ROCKET_STATE RocketChannel::poweredAscent(uint64_t time)
{
	if (time - timeLastTransition > FLIGHT_BURN_TIME)
	{ // motor burnout, close valves, IMPORTANT!: total burn time before shutoff is powered + unpowered ascent
		return UNPOWERED_ASCENT;
	}
	return POWERED_ASCENT;
}

ROCKET_STATE RocketChannel::depress(uint64_t time)
{
	if (GetSensorReading(oxPressureChannel) < 1.5 && GetSensorReading(fuelPressureChannel) < 1.5)
	{

		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURE_CONTROLLER, PI_CONTROL_ENABLED, 0);
		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_PRESSURANT_VALVE, SERVO_TARGET_POSITION, 65535);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_PRESSURANT_VALVE, SERVO_TARGET_POSITION, 65535);
		can.SetRemoteVariable(DEVICE_ID_FUEL_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 0);
		can.SetRemoteVariable(DEVICE_ID_OX_ECU_VENT_VALVE, DIGITAL_OUT_STATE, 0);


		return PAD_IDLE;
	}
	return DEPRESS;
}

ROCKET_STATE RocketChannel::abort(uint64_t time)
{
	return (is_main_ecu) ? ABORT : PAD_IDLE;
}
ROCKET_STATE RocketChannel::pressurize_tanks(uint64_t time)
{
	if (time - timeLastTransition > 55000)
	{
		return ABORT;
	}
	return PRESSURIZE_TANKS;
}

int RocketChannel::reset()
{
	externalNextState = PAD_IDLE;
	internalNextState = PAD_IDLE;
	ignitionState = IgnitionSequence::INIT;
	sensor_slope = 0.01888275146;
	sensor_offset = -15;
	chamberPressureMin = 0;
	chamberPressureLowCounter = 0;
	chamberPressureGoodCounter = 0;
	autoCheckBadCounter = 0;
	state = PAD_IDLE;
	return 0;
}

int RocketChannel::processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n)
{
	switch (commandId)
	{
	case ROCKET_REQ_INTERNAL_CONTROL:
		if (state == PAD_IDLE)
		{
			externalNextState = (is_main_ecu) ? IGNITION_SEQUENCE : PRESSURIZE_TANKS;
		}
		return 0;
	case ROCKET_REQ_ABORT:
		externalNextState = ABORT;
		return 0;
	case ROCKET_REQ_END_OF_FLIGHT:
		if (state == UNPOWERED_ASCENT)
		{
			externalNextState = DEPRESS;
		}
		return 0;
	case ROCKET_REQ_SET_ROCKET_STATE:
		setRocketState(returnData, n);
		return 0;
	case ROCKET_REQ_GET_ROCKET_STATE:
		getRocketState(returnData, n);
		return 0;
	case ROCKET_REQ_AUTO_CHECK:
		if (state == PAD_IDLE)
		{
			externalNextState = AUTO_CHECK;
		}
		return 0;
	default:
		return AbstractChannel::processMessage(commandId, returnData, n);
	}
}

int RocketChannel::getSensorData(uint8_t *data, uint8_t &n)
{
	uint16_t *out = (uint16_t*) (data + n);
	*out = state;

	n += ROCKET_DATA_N_BYTES;
	return 0;
}

int RocketChannel::setVariable(uint8_t variableId, int32_t data)
{
	switch (variableId)
	{
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

int RocketChannel::getVariable(uint8_t variableId, int32_t &data) const
{
	switch (variableId)
	{
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

void RocketChannel::setRocketState(uint8_t *data, uint8_t &n)
{
	RocketStateReqMsg_t *rocketStateRequestMsg;
	RocketStateResMsg_t *rocketStateResponseMsg;
	rocketStateRequestMsg = (RocketStateReqMsg_t*) data;
	externalNextState = static_cast<ROCKET_STATE>(rocketStateRequestMsg->state);

	rocketStateResponseMsg = (RocketStateResMsg_t*) data;
	rocketStateResponseMsg->state = state;
	rocketStateResponseMsg->status = SUCCESS;
	n += sizeof(RocketStateResMsg_t);
}

void RocketChannel::getRocketState(uint8_t *data, uint8_t &n)
{
	RocketStateResMsg_t *rocketStateResponseMsg;

	rocketStateResponseMsg = (RocketStateResMsg_t*) data;
	rocketStateResponseMsg->state = state;
	rocketStateResponseMsg->status = WRITABLE;
	n += sizeof(RocketStateResMsg_t);
}

void RocketChannel::SetRemoteRocketState(DeviceIds device_id, ROCKET_CMDs state)
{
	uint8_t empty_buffer[10] =
	{ 0 };
	can.sendAsMaster(device_id, state, empty_buffer, sizeof(uint32_t));
}
double RocketChannel::GetSensorReading(const ADCChannel& sensor_channel)
{
	uint16_t data = sensor_channel.getMeasurement();
	return ((double) data * sensor_slope + sensor_offset);
}
