#include <Channels/CANMonitorChannel.h>
#include <cstring>
#include <cstdio>
#include <STRHAL.h>

CANMonitorChannel::CANMonitorChannel(uint8_t id, STRHAL_FDCAN_Id_t fdcan_id, uint32_t refreshDivider) :
		AbstractChannel(CHANNEL_TYPE_CAN_MONITOR, id, refreshDivider), fdcan_id(fdcan_id)
{
}

int CANMonitorChannel::init()
{
	if (fdcan_id == STRHAL_N_FDCAN)
		return -1;

	return 0;
}

int CANMonitorChannel::exec()
{
	FDCAN_StatusRegisters_t status = {};
	status.raw.psr = STRHAL_CAN_Read_PSR_Reg(fdcan_id)&((1<<PSR_SIZE)-1);
	if (lec ==  0 && status.bits.LEC !=0b111)
	{
		lec = status.bits.LEC;
	}
	if (dlec ==  0 && status.bits.DLEC !=0b111)
	{
		dlec = status.bits.DLEC;
	}
	return 0;
}

int CANMonitorChannel::reset()
{
	return 0;
}

int CANMonitorChannel::processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n)
{
	switch (commandId)
	{
		default:
			return AbstractChannel::processMessage(commandId, returnData, n);
	}
}

int CANMonitorChannel::getSensorData(uint8_t *data, uint8_t &n)
{
  	auto out = (FDCAN_StatusRegisters_t*) (data + n);
	n += CAN_MONITOR_DATA_N_BYTES;

    out->raw.ecr = STRHAL_CAN_Read_ECR_Reg(fdcan_id)&((1<<ECR_SIZE)-1);
    out->raw.psr = STRHAL_CAN_Read_PSR_Reg(fdcan_id)&((1<<PSR_SIZE)-1);
	out->bits.LEC = lec;
	out->bits.DLEC = lec;
    return 0;
}

int CANMonitorChannel::setVariable(uint8_t variableId, int32_t data)
{
	switch (variableId)
	{
		case CAN_MONITOR_REFRESH_DIVIDER:
			refreshDivider = data;
			refreshCounter = 0;
			return 0;
		default:
			return -1;
	}
}

int CANMonitorChannel::getVariable(uint8_t variableId, int32_t &data) const
{
	switch (variableId)
	{
		case CAN_MONITOR_REFRESH_DIVIDER:
			data = (int32_t) refreshDivider;
			return 0;
		default:
			return -1;
	}
}
