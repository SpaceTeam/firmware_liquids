#ifndef PICONTROLCHANNEL_H
#define PICONTROLCHANNEL_H

#include "./Channels/AbstractChannel.h"
#include "./Channels/GenericChannel.h"
#include "./Channels/ServoChannel.h"
#include "./Channels/ADCChannel.h"
#include <can_houbolt/channels/pi_control_channel_def.h>
#include <STRHAL.h>

class PIControlChannel: public AbstractChannel
{
	public:
		PIControlChannel(uint8_t id, GenericChannel &parent, uint8_t inputChannelId, ServoChannel& ServoChannel, uint32_t refreshDivider);

		PIControlChannel(const PIControlChannel &other) = delete;
		PIControlChannel& operator=(const PIControlChannel &other) = delete;
		PIControlChannel(const PIControlChannel &&other) = delete;
		PIControlChannel& operator=(const PIControlChannel &&other) = delete;

		int init() override;
		int reset() override;
		int exec() override;
		int getSensorData(uint8_t *data, uint8_t &n) override;

		int processMessage(uint8_t commandId, uint8_t *returnData, uint8_t &n) override;

		void setEnabled(uint8_t state);

		static constexpr uint16_t EXEC_SAMPLE_TICKS = 20;

	protected:

		int setVariable(uint8_t variableId, int32_t data) override;
		int getVariable(uint8_t variableId, int32_t &data) const override;

	private:
		uint16_t enabled = 0;
		double targetPressure = 30;
		double p_pos_gain = 0;
		double i_pos_gain = 0;
		double p_neg_gain = 0;
		double i_neg_gain = 0;
		double integral_term = 0;
	    double last_error = 0;
	    double sensor_slope = 0.01888275146;
	    double sensor_offset = -15;
	    double operating_point = 0;

		GenericChannel &parent;
		uint8_t inputChannelId;
		ServoChannel &servoChannel;

		uint64_t last_sample_time = 0;

};

#endif /*PICONTROLCHANNEL_H*/
