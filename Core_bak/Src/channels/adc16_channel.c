#include "adc16_channel.h"
#include "main.h"
#include "channel_util.h"
#include "channels.h"
#include "ui.h"

#include "serial.h"
#include <string.h>
#include <stdio.h>

Result_t Adc16_ResetSettings(Channel_t *channel);
Result_t Adc16_Status(Channel_t *channel);
Result_t Adc16_SetVariable(Channel_t *channel, SetMsg_t *set_msg);
Result_t Adc16_GetVariable(Channel_t *channel, GetMsg_t *get_msg, ADC16_CMDs response_cmd);

static uint16_t readonly_var = 0;
uint16_t* Adc16_VariableSelection(Adc16_Channel_t *adc16, uint8_t var_id, uint8_t ch_id)
{
	readonly_var = 0;
	switch (var_id)
	{
		case ADC16_MEASUREMENT:
			readonly_var = *adc16->analog_in;
			///readonly_var = (adc16->is_single) ? adc16->data : *adc16->analog_in;
			return &readonly_var;
		case ADC16_REFRESH_DIVIDER:
			return NULL;
		default:
			return NULL;
	}
}

Result_t Adc16_ResetSettings(Channel_t *channel)
{

	return OOF;
}

Result_t Adc16_Status(Channel_t *channel)
{

	return OOF;
}

Result_t Adc16_SetVariable(Channel_t *channel, SetMsg_t *set_msg)
{
	if (set_msg->variable_id == ADC16_MEASUREMENT) //cannot set state, read-only
		return OOF;

	uint16_t *var = Adc16_VariableSelection(&channel->channel.adc16, set_msg->variable_id, channel->id);
	if (var == NULL)
		return OOF;
	*var = set_msg->value;
	return Adc16_GetVariable(channel, (GetMsg_t*) set_msg, ADC16_RES_SET_VARIABLE);
}

Result_t Adc16_GetVariable(Channel_t *channel, GetMsg_t *get_msg, ADC16_CMDs response_cmd)
{
	return ChannelUtil_GetVariable(channel, get_msg, response_cmd);
}

Result_t Adc16_ProcessMessage(uint8_t ch_id, uint8_t cmd_id, uint8_t *data, uint32_t length)
{

	if (node.channels[ch_id].type != CHANNEL_TYPE_ADC16)
		return OOF_WRONG_CHANNEL_TYPE;
	Channel_t *channel = &node.channels[ch_id];

	switch (cmd_id)
	{
		case ADC16_REQ_RESET_SETTINGS:
			return Adc16_ResetSettings(channel);
		case ADC16_REQ_STATUS:
			return Adc16_Status(channel);
		case ADC16_REQ_SET_VARIABLE:
			return Adc16_SetVariable(channel, (SetMsg_t*) data);
		case ADC16_REQ_GET_VARIABLE:
			return Adc16_GetVariable(channel, (GetMsg_t*) data, ADC16_RES_GET_VARIABLE);
		case ADC16_REQ_CALIBRATE:
			return OOF;
		default:
			return OOF_UNKNOWN_CMD;
	}
}

Result_t Adc16_GetRawData(uint8_t channel_id, uint16_t *data)
{
	*data = (uint16_t) *node.channels[channel_id].channel.adc16.analog_in;
	//TODO @ANDI if (No new data and/or refresh divider count)  return OOF_NO_NEW_DATA;
	return NOICE;
}

Result_t Adc16_GetData(uint8_t ch_id, uint8_t *data, uint32_t *length)
{
	uint16_t *out = (uint16_t*) (data + *length);
	uint16_t new_data;
	Result_t result = Adc16_GetRawData(ch_id, &new_data);

	if (result != NOICE)
		return result;
	*out = new_data;
#ifdef DEBUG_DATA
	Serial_PutInt(new_data);
	Serial_PutString(", ");
#endif
	(*length) += ADC16_DATA_N_BYTES;
	return NOICE;
}