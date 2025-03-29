#include <Channels/PIControlChannel.h>
#include <STRHAL.h>

#include <cstdio>
#include <cstring>
PIControlChannel::PIControlChannel(uint8_t id, GenericChannel &parent,
		uint8_t inputChannelId, ServoChannel &servoChannel,
		uint32_t refreshDivider) :
		AbstractChannel(CHANNEL_TYPE_PI_CONTROL, id, refreshDivider), parent(
				parent), inputChannelId(inputChannelId), servoChannel(
				servoChannel)
{
}

int PIControlChannel::init()
{

	enabled = 0;
	targetPressure = 32;
	p_pos_gain = 1;
	i_pos_gain = 1;
	p_neg_gain = 1;
	i_neg_gain = 1;
	integral_term = 0;
	last_error = 0;
	sensor_slope = 0.01888275146;
	sensor_offset = -15;
	operating_point = 40;
	return 0;
}

int PIControlChannel::exec()
{

	//static uint8_t debug_counter = 0;
	//STRHAL_GPIO_t debug_led = { GPIOC, 9, STRHAL_GPIO_TYPE_OPP };
	if (enabled == 0)
	{
		return 0;
	}

	uint64_t time = STRHAL_Systick_GetTick();
	if ((time - last_sample_time) < EXEC_SAMPLE_TICKS)
		return 0;
	//STRHAL_GPIO_Write(&debug_led, STRHAL_GPIO_VALUE_H);
	double sample_time = (double) (time - last_sample_time) / 1000;

	last_sample_time = time;

	double pressure_raw =
			(parent.getControlInputChannel(inputChannelId))->getMeasurement(); //pressureChannel.getMeasurement();
	double pressure = pressure_raw * sensor_slope + sensor_offset;

	double error = targetPressure - pressure;

	double p_gain = (error > 0) ? p_pos_gain : p_neg_gain;
	double i_gain = (error > 0) ? i_pos_gain : i_neg_gain;

	integral_term += i_gain * error * sample_time;

	double new_output = p_gain * error + integral_term;
	//last_error = error;
	new_output = new_output + operating_point;

	if (new_output > 100.0)
	{
		new_output = 100.0;
		integral_term -= i_gain * error * sample_time;
	}
	else if (new_output < 0.0)
	{
		new_output = 0.0;
		integral_term -= i_gain * error * sample_time;
	}
	uint16_t output = (uint16_t) (UINT16_MAX * new_output / 100.0);
	servoChannel.setTargetPos(output);
	//STRHAL_GPIO_Write(&debug_led, STRHAL_GPIO_VALUE_L);

	/*
	 debug_counter++;
	 if(	debug_counter > 100)
	 {

	 char buf[100] = "";
	 sprintf(buf, "%lf, %lf, %lf, %lf, %lf\n", pressure, targetPressure, error, integral_term, new_output);
	 STRHAL_UART_Debug_Write_DMA(buf, strlen(buf));

	 debug_counter = 0;
	 }
	 */
	return 0;
}

int PIControlChannel::reset()
{
	return 0;
}

int PIControlChannel::processMessage(uint8_t commandId, uint8_t *returnData,
		uint8_t &n)
{
	switch (commandId)
	{
	default:
		return AbstractChannel::processMessage(commandId, returnData, n);
	}
}
void PIControlChannel::setEnabled(uint8_t state)
{
	last_sample_time = STRHAL_Systick_GetTick();
	enabled = (state != 0);
	if (enabled == 0)
	{
		integral_term = 0;
		last_error = 0;
		last_sample_time = 0;
		servoChannel.setTargetPos(0);
	}
}

int PIControlChannel::getSensorData(uint8_t *data, uint8_t &n)
{
	uint16_t *out = (uint16_t*) (data + n);
	*out = (uint16_t) servoChannel.getState();

	n += PI_CONTROL_DATA_N_BYTES;
	return 0;
}

int PIControlChannel::setVariable(uint8_t variableId, int32_t data)
{

	switch (variableId)
	{
	case PI_CONTROL_ENABLED:
		enabled = (data != 0);
		setEnabled(enabled);
		return 0;
	case PI_CONTROL_TARGET:
		targetPressure = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_P_POS:
		p_pos_gain = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_I_POS:
		i_pos_gain = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_P_NEG:
		p_neg_gain = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_I_NEG:
		i_neg_gain = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_SENSOR_SLOPE:
		sensor_slope = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_SENSOR_OFFSET:
		sensor_offset = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_OPERATING_POINT:
		operating_point = (double) data / 1000.0;
		return 0;
	case PI_CONTROL_ACTUATOR_CHANNEL_ID:
		return -1;
	case PI_CONTROL_SENSOR_CHANNEL_ID:
		inputChannelId = data;
		return 0;
	case PI_CONTROL_REFRESH_DIVIDER:
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
	case PI_CONTROL_ENABLED:
		data = enabled;
		return 0;
	case PI_CONTROL_TARGET:
		data = (int32_t) (targetPressure * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_P_POS:
		data = (int32_t) (p_pos_gain * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_I_POS:
		data = (int32_t) (i_pos_gain * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_P_NEG:
		data = (int32_t) (p_neg_gain * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_I_NEG:
		data = (int32_t) (i_neg_gain * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_SENSOR_SLOPE:
		data = (int32_t) (sensor_slope * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_SENSOR_OFFSET:
		data = (int32_t) (sensor_offset * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_OPERATING_POINT:
		data = (int32_t) (operating_point * 1000); // convert back to 16bit full scale
		return 0;
	case PI_CONTROL_ACTUATOR_CHANNEL_ID:
		data = servoChannel.getChannelId();
		return 0;
	case PI_CONTROL_SENSOR_CHANNEL_ID:
		data = (int32_t) inputChannelId;
		return 0;
	case PI_CONTROL_REFRESH_DIVIDER:
		data = (int32_t) refreshDivider;
		return 0;
	default:
		return -1;
	}
}
