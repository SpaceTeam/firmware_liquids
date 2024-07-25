#include "../Inc/Channels/TempChannel.h"

#include <cstring>
#include <cstdio>

TempChannel::TempChannel(uint8_t id, MAX31865_Temp *temp, uint32_t refreshDivider) :
		AbstractChannel(CHANNEL_TYPE_ADC16, id, refreshDivider), temp(temp)
{
}

int TempChannel::init()
{
	return 0;
}

int TempChannel::exec()
{
	return 0;
}

int TempChannel::reset()
{
	return temp->reset();
}

int TempChannel::getSensorData(uint8_t *data, uint8_t &n)
{
	if (temp->new_data)
	{
		uint8_t *out = data + n;
		uint16_t measurement = temp->measurementData;
		temp->new_data = 0;
		//baro.getMeasurement(measurement);
		out[0] = (uint8_t) (measurement >> 0) & 0xFF;
		out[1] = (uint8_t) (measurement >> 8) & 0xFF;

		n += ADC16_DATA_N_BYTES;
		return 0;
	}

	return -1;
}

int TempChannel::processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n)
{
	return AbstractChannel::processMessage(commandId, returnData, n);
}

int TempChannel::setVariable(uint8_t variableId, int32_t data)
{
	switch (variableId)
	{
		case ADC16_REFRESH_DIVIDER:
			refreshDivider = data;
			refreshCounter = 0;
			return 0;
		default:
			return -1;
	}
}

int TempChannel::getVariable(uint8_t variableId, int32_t &data) const
{
	switch (variableId)
	{
		case ADC16_REFRESH_DIVIDER:
			data = (int32_t) refreshDivider;
			return 0;
		default:
			return -1;
	}
}

uint16_t TempChannel::getMeasurement() const
{
	return temp->measurementData;
}
TempChannel::~TempChannel()
{

}
