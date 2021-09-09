#include "threshold.h"
#include "channels.h"

Result_t CheckThresholdComparison(uint8_t channel_id, Threshold_t *threshold_ptr)
{
	uint32_t value = 0;
	switch(node.channels[channel_id].type)
	{
		case CHANNEL_TYPE_ADC16:
			value = (uint32_t)*Adc16_VariableSelection(&node.channels[channel_id].channel.adc16, threshold_ptr->var_id, channel_id);
			break;
		case CHANNEL_TYPE_ADC16_SINGLE:
			value = (uint32_t)*Adc16Single_VariableSelection(&node.channels[channel_id].channel.adc16single, threshold_ptr->var_id, channel_id);
			break;
		case CHANNEL_TYPE_ADC24:
			value = (uint32_t)*Adc24_VariableSelection(&node.channels[channel_id].channel.adc24, threshold_ptr->var_id);
			break;
		case CHANNEL_TYPE_DIGITAL_OUT:
			value = (uint32_t)*DigitalOut_VariableSelection(&node.channels[channel_id].channel.digital_out, threshold_ptr->var_id, channel_id);
			break;
		case CHANNEL_TYPE_PNEUMATIC_VALVE:
			value = *PneumaticValve_VariableSelection(&node.channels[channel_id].channel.pneumatic_valve, threshold_ptr->var_id, channel_id);
			break;
		case CHANNEL_TYPE_SERVO:
			value = *Servo_VariableSelection(&node.channels[channel_id].channel.servo, threshold_ptr->var_id, channel_id);
			break;
		case CHANNEL_TYPE_NODE_GENERIC:
			return OOF_NO_OP;
		default:
			return OOF_NO_OP;
	}
	switch(threshold_ptr->compare_id)
	{
		case EQUALS:
			return (value == threshold_ptr->threshold) ? threshold_ptr->result : NOICE;
		case LESS_THAN:
			return (value < threshold_ptr->threshold) ? threshold_ptr->result : NOICE;
		case GREATER_THAN:
			return (value > threshold_ptr->threshold) ? threshold_ptr->result : NOICE;
		default:
			return OOF_NO_OP;
	}
}


Result_t CheckThreshold(uint8_t channel_id, uint8_t threshold_id)
{

	Threshold_t * threshold_ptr = &channel_thresholds[channel_id][threshold_id];

	Result_t result = CheckThresholdComparison(channel_id, threshold_ptr);

	uint8_t temp_result = 1;
	if( channel_id != threshold_ptr->or_threshold_id )
		temp_result = CheckThreshold(channel_id, threshold_ptr->or_threshold_id) == NOICE;

	if( channel_id != threshold_ptr->and_threshold_id )
		temp_result = temp_result && (CheckThreshold(channel_id, threshold_ptr->and_threshold_id) == NOICE);

	return (temp_result) ? result : OOF;
}

Result_t CheckThresholds(uint8_t channel_id)
{
	for( int i = 0; i < MAX_THRESHOLDS; i++)
	{
		Threshold_t threshold_struct = channel_thresholds[channel_id][i];
		if( threshold_struct.enabled >= 0 )
		{
			Result_t result = CheckThreshold(channel_id, i);
			if( result != NOICE)
				return result;

		}
	}
	return NOICE;
}
