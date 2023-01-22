#ifndef CANCOM_H
#define CANCOM_H

class GenericChannel; // Forward declaration to avoid cyclic dependency

#include "AbstractCOM.h"
#include "Channels/AbstractChannel.h"
#include "STRHAL.h"

class CANCOM : public AbstractCOM {
	public:
		CANCOM(const CANCOM &other) = delete;
		CANCOM& operator=(const CANCOM &other) = delete;
		CANCOM(const CANCOM &&other) = delete;
		CANCOM& operator=(const CANCOM &&other) = delete;
		~CANCOM();

		static CANCOM* instance(GenericChannel *genericChannel = nullptr);

		COMState init() override;

		COMState exec() override;

		COMState init(COMMode mode);
		void sendAsMaster(uint8_t receiverNodeId, uint8_t receiverChannelId, uint8_t commandId, uint8_t *data, uint8_t n);

		COMState subscribe2Node(uint8_t nodeId, AbstractChannel & channel);

	private:
		CANCOM(GenericChannel *genericChannel);

		static GenericChannel *genericChannel;
		static void standardReceptor(uint32_t id, uint8_t *data, uint32_t n);
		static void bridgeReceptor(STRHAL_FDCAN_Id_t bus_id, uint32_t id, uint8_t *data, uint32_t n);
		static void internalReceptor(uint32_t id, uint8_t *data, uint32_t n);
		static void externalReceptor(uint32_t id, uint8_t *data, uint32_t n);
		static void heartbeat();

		static CANCOM *cancom;

};

#endif /*CANCOM_H*/