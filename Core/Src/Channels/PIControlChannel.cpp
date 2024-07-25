#include <Channels/PIControlChannel.h>

PIControlChannel::PIControlChannel(uint8_t id, GenericChannel &parent, uint8_t inputChannelId, ServoChannel &servoChannel, uint32_t refreshDivider) :
		AbstractChannel(CHANNEL_TYPE_CONTROL, id, refreshDivider), parent(parent), inputChannelId(inputChannelId), servoChannel(servoChannel)
{
}

int PIControlChannel::init()
{

	return 0;
}

int PIControlChannel::exec()
{

	static double gain = 10;
	static double offset = -10;

	if (enabled == 0)
	{

		integral_term = 0;
		last_error = 0;
		last_sample_time = 0;
		servoChannel.setTargetPos(0);

		return 0;
	}

	uint64_t time = STRHAL_Systick_GetTick();
	if ((time - last_sample_time) < EXEC_SAMPLE_TICKS)
		return 0;

	double sample_time = (double) (time - last_sample_time) / 1000;

	last_sample_time = time;

	double pressure = (parent.getControlInputChannel(inputChannelId))->getMeasurement(); //pressureChannel.getMeasurement();

	pressure = pressure * gain + offset;

	double error = targetPressure - pressure;
	integral_term += i_gain * error * sample_time;

	double new_output = p_gain * error + integral_term;
	last_error = error;
	//pressurant_valve = output + obj.Operating_Point;

	new_output = (new_output > 100.0) ? 100.0 : new_output;
	new_output = (new_output < 0.0) ? 0.0 : new_output;
	uint16_t output = (uint16_t) (UINT16_MAX * new_output / 100.0);
	servoChannel.setTargetPos(output);
	return 0;
}

int PIControlChannel::reset()
{
	return 0;
}

int PIControlChannel::processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n)
{
	switch (commandId)
	{
		default:
			return AbstractChannel::processMessage(commandId, returnData, n);
	}
}

int PIControlChannel::getSensorData(uint8_t *data, uint8_t &n)
{
	uint16_t *out = (uint16_t*) (data + n);
	*out = (uint16_t) servoChannel.getState();

	n += CONTROL_DATA_N_BYTES;
	return 0;
}

int PIControlChannel::setVariable(uint8_t variableId, int32_t data)
{
	switch (variableId)
	{
		case CONTROL_ENABLED:
			enabled = (data != 0);
			if (data == 0 && servoChannel.getState() != 0)
			{
				if (servoChannel.setState(0) != 0) // make sure servo is closed after disabling
					return -1;
			}
			return 0;
		case CONTROL_TARGET:
			targetPressure = (double) data / 1000.0;
			return 0;
		case CONTROL_THRESHOLD: //used as P gain
			p_gain = (double) data / 1000.0;
			return 0;
		case CONTROL_HYSTERESIS: // used as I gain
			i_gain = (double) data / 1000.0;
			return 0;
		case CONTROL_ACTUATOR_CHANNEL_ID:
			return -1;
		case CONTROL_SENSOR_CHANNEL_ID:
			inputChannelId = data;
			return 0;
		case CONTROL_REFRESH_DIVIDER:
			refreshDivider = data;
			refreshCounter = 0;
			return 0;
		default:
			return -1;
	}
}

int PIControlChannel::getVariable(uint8_t variableId, int32_t &data) const
{
	switch (variableId)
	{
		case CONTROL_ENABLED:
			data = enabled;
			return 0;
		case CONTROL_TARGET:
			data = (int32_t) (targetPressure * 1000); // convert back to 16bit full scale
			return 0;
		case CONTROL_THRESHOLD:
			data = (int32_t) (p_gain * 1000); // convert back to 16bit full scale
			return 0;
		case CONTROL_HYSTERESIS:
			data = (int32_t) (i_gain * 1000); // convert back to 16bit full scale
			return 0;
		case CONTROL_ACTUATOR_CHANNEL_ID:
			data = servoChannel.getChannelId();
			return 0;
		case CONTROL_SENSOR_CHANNEL_ID:
			data = (int32_t) inputChannelId;
			return 0;
		case CONTROL_REFRESH_DIVIDER:
			data = (int32_t) refreshDivider;
			return 0;
		default:
			return -1;
	}
}
