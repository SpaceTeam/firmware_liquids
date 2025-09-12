#ifndef RADIO_H
#define RADIO_H

#include "Channels/AbstractChannel.h"
#include "Modules/SX1276.h"
#include <AbstractCom.h>
#include "STRHAL.h"

class Radio: public AbstractCom
{
	public:
		Radio(const Radio &other) = delete;
		Radio& operator=(const Radio &other) = delete;
		Radio(const Radio &&other) = delete;
		Radio& operator=(const Radio &&other) = delete;

		static Radio& instance(uint32_t nodeId, SX1276 &lora);

		int init(Com_Receptor_t receptor, Com_Heartbeat_t heartbeat) override;
		int exec() override;

		static int send(uint32_t id, uint8_t *data, uint8_t n);

		static constexpr uint8_t ENGINE_ECU_START_ADDR = 0;
		static constexpr uint8_t ENGINE_ECU_MSG_SIZE = 47;
		static constexpr uint8_t FUEL_ECU_START_ADDR = ENGINE_ECU_START_ADDR + ENGINE_ECU_MSG_SIZE;
		static constexpr uint8_t FUEL_ECU_MSG_SIZE = 47;
		static constexpr uint8_t OX_ECU_START_ADDR = FUEL_ECU_START_ADDR + FUEL_ECU_MSG_SIZE;
		static constexpr uint8_t OX_ECU_MSG_SIZE = 47;
		static constexpr uint8_t RCU_START_ADDR = OX_ECU_START_ADDR + OX_ECU_MSG_SIZE;
		static constexpr uint8_t RCU_MSG_SIZE = 44;
		static constexpr uint32_t MSG_SIZE = ENGINE_ECU_MSG_SIZE + FUEL_ECU_MSG_SIZE + OX_ECU_MSG_SIZE + RCU_MSG_SIZE; //185
		static uint8_t msgArray[MSG_SIZE];

	private:
		static SX1276 *lora;
		Radio(uint32_t nodeId, SX1276 &lora);
};

#endif /*RADIO_H*/
