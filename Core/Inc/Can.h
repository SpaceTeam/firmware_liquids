#ifndef CAN_H
#define CAN_H

#include "Channels/AbstractChannel.h"
#include "STRHAL.h"
#include <AbstractCom.h>
#include <tuple>
#include "NodeInfos.h"
#include "RingBuf.h"

//#include <Communication.h>
#define MAIN_CAN_BUS STRHAL_FDCAN1

class Can: public AbstractCom
{
	public:
		Can(const Can &other) = delete;
		Can& operator=(const Can &other) = delete;
		Can(const Can &&other) = delete;
		Can& operator=(const Can &&other) = delete;

		static Can& instance(uint32_t nodeId = 0);

		int init(Com_Receptor_t receptor, Com_Heartbeat_t heartbeat) override;
		int init(Com_Receptor_t receptor, Com_Heartbeat_t heartbeat, COMMode mode);
		int exec() override;

		static int send(uint32_t id, uint8_t *data, uint8_t n);
		void SetRemoteVariable(DeviceIds device_id, uint8_t variable_id, int32_t value);
		void sendAsMaster(uint8_t receiverNodeId, uint8_t receiverChannelId, uint8_t commandId, uint8_t *data, uint8_t n);
		void sendAsMaster(DeviceIds device_id, uint8_t commandId, uint8_t *data, uint8_t n);

	private:
		Can(uint32_t nodeId);

        static Can* canPtr;

		static void bridgeReceptor(STRHAL_FDCAN_Id_t bus_id, uint32_t id, uint8_t *data, uint32_t n);
		static void internalReceptor(uint32_t id, uint8_t *data, uint32_t n);
		static void externalReceptor(uint32_t id, uint8_t *data, uint32_t n);
        static void bufferingReceptor(uint32_t id, uint8_t *data, uint32_t n);

		static Com_Receptor_t standardReceptor;
		static uint32_t _nodeId;
        RingBuf<std::tuple<Can_MessageId_t, Can_MessageData_t, uint32_t>, 16> canBuf;
};

#endif /*CAN_H*/
