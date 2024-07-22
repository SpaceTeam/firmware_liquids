#ifndef TEMPCHANNEL_H
#define TEMPCHANNEL_H

#include "./Channels/AbstractChannel.h"
#include <can_houbolt/channels/adc16_channel_def.h>
#include "../Modules/MAX31865_Temp.h"
#include <STRHAL.h>

class TempChannel: public AbstractChannel
{
	public:
		TempChannel(uint8_t id, MAX31865_Temp *temp, uint32_t refreshDivider);

		TempChannel(const TempChannel &other) = delete;
		TempChannel& operator=(const TempChannel &other) = delete;
		TempChannel(const TempChannel &&other) = delete;
		TempChannel& operator=(const TempChannel &&other) = delete;

		~TempChannel();

		int init() override;
		int reset() override;
		int exec() override;

		int getSensorData(uint8_t *data, uint8_t &n) override;

		int processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) override;

	protected:
		int setVariable(uint8_t variableId, int32_t data) override;
		int getVariable(uint8_t variableId, int32_t &data) const override;
	private:
		MAX31865_Temp *temp;
};

#endif /*TEMPCHANNEL_H*/
