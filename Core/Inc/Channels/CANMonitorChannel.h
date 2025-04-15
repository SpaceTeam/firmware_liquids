#ifndef CANMONITORCHANNEL
#define CANMONITORCHANNEL

#include "./Channels/AbstractChannel.h"
#include <can_houbolt/channels/can_monitor_channel_def.h>
#include <STRHAL.h>

class CANMonitorChannel: public AbstractChannel
{
	public:
		CANMonitorChannel(uint8_t id, STRHAL_FDCAN_Id_t fdcan_id, uint32_t refreshDivider);

		CANMonitorChannel(const CANMonitorChannel &other) = delete;
		CANMonitorChannel& operator=(const CANMonitorChannel &other) = delete;
		CANMonitorChannel(const CANMonitorChannel &&other) = delete;
		CANMonitorChannel& operator=(const CANMonitorChannel &&other) = delete;

		int init() override;
		int reset() override;
		int exec() override;
		int getSensorData(uint8_t *data, uint8_t &n) override;

		int processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) override;

	protected:

		int setVariable(uint8_t variableId, int32_t data) override;
		int getVariable(uint8_t variableId, int32_t &data) const override;

	private:
		STRHAL_FDCAN_Id_t fdcan_id;
		int lec = 0;
		int dlec = 0;
};

#endif CANMONITORCHANNEL
