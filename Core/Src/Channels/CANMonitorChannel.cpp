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

	// We store the lec/dlec values because they are reset after each successful transmission.
	// Should we only have intermittent failures, we can then still report one of these errors.
	uint32_t psr = STRHAL_CAN_Read_PSR_Reg(fdcan_id)&((1<<PSR_SIZE)-1);
	if (lec ==  0 && FDCAN_StatusRegisters::PSR_Fields::LEC.get(psr) !=0b111)
	{
		lec = FDCAN_StatusRegisters::PSR_Fields::LEC.get(psr);
	}
	if (dlec ==  0 && FDCAN_StatusRegisters::PSR_Fields::DLEC.get(psr) !=0b111)
	{
		dlec = FDCAN_StatusRegisters::PSR_Fields::LEC.get(psr);
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
	n += CAN_MONITOR_DATA_N_BYTES;

	uint32_t ecr = STRHAL_CAN_Read_ECR_Reg(fdcan_id)&((1<<ECR_SIZE)-1);
	uint32_t psr = STRHAL_CAN_Read_PSR_Reg(fdcan_id)&((1<<PSR_SIZE)-1);
	uint64_t can_data = 0;
	FDCAN_StatusRegisters::ECR.set(can_data,ecr);
	FDCAN_StatusRegisters::PSR.set(can_data,psr);
	FDCAN_StatusRegisters::PSR_Fields::LEC.set(can_data,lec);
	FDCAN_StatusRegisters::PSR_Fields::DLEC.set(can_data,dlec);
	memcpy(data,&can_data,CAN_MONITOR_DATA_N_BYTES);
	char buf[32];
	sprintf(buf,"REC: %i, TEC: %i \r\n", FDCAN_StatusRegisters::ECR_Fields::REC.get(can_data) ,FDCAN_StatusRegisters::ECR_Fields::TEC.get(can_data));
	STRHAL_UART_Debug_Write_DMA(buf, strlen(buf));

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
