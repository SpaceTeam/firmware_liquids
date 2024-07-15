#include "../Inc/ECU_Lamarr.h"

#include <cstdio>
#include <cstring>
#include "stddef.h"
ECU_Lamarr::ECU_Lamarr(uint32_t node_id, uint32_t fw_version, uint32_t refresh_divider) :
		GenericChannel(node_id, fw_version, refresh_divider),
		led_1({ GPIOC, 8, STRHAL_GPIO_TYPE_OPP }),
		led_2({ GPIOC, 9, STRHAL_GPIO_TYPE_OPP }),
		press_0(0,{ ADC1, STRHAL_ADC_CHANNEL_15 }, 1),
		press_1(1,{ ADC1, STRHAL_ADC_CHANNEL_12 }, 1),
		press_2(2,{ ADC3, STRHAL_ADC_CHANNEL_4 }, 1),
		press_3(3,{ ADC3, STRHAL_ADC_CHANNEL_6 }, 1),
		temp_0(4,{ ADC2, STRHAL_ADC_CHANNEL_5 }, 1),
		temp_1(5,{ ADC2, STRHAL_ADC_CHANNEL_17 }, 1),
		servo_0(6, 0, STRHAL_TIM_TIM2, STRHAL_TIM_TIM2_CH1_PA0,{ ADC1, STRHAL_ADC_CHANNEL_2 },{ NULL, STRHAL_ADC_CHANNEL_LAST },{ GPIOC, 13, STRHAL_GPIO_TYPE_OPP }, 1),
		servo_1(7, 1, STRHAL_TIM_TIM2, STRHAL_TIM_TIM2_CH3_PA2,{ ADC1, STRHAL_ADC_CHANNEL_4 },{ NULL, STRHAL_ADC_CHANNEL_LAST },{ GPIOC, 14, STRHAL_GPIO_TYPE_OPP }, 1),
		pyro0_cont(9,{ GPIOA, 15, STRHAL_GPIO_TYPE_IHZ }, 1),
		pyro1_cont(11,{ GPIOC, 12, STRHAL_GPIO_TYPE_IHZ }, 1),
		pyro2_cont(13,{ GPIOD, 0, STRHAL_GPIO_TYPE_IHZ }, 1),
		pyro3_cont(15,{ GPIOB, 3, STRHAL_GPIO_TYPE_IHZ }, 1),
		pyro_igniter0(8,{ ADC1, STRHAL_ADC_CHANNEL_6 },{ GPIOC, 10, STRHAL_GPIO_TYPE_OPP },pyro0_cont, 1),
		pyro_igniter1(10,{ ADC1, STRHAL_ADC_CHANNEL_7 },{ GPIOC, 11, STRHAL_GPIO_TYPE_OPP }, pyro1_cont, 1),
		pyro_igniter2(12,{ ADC1, STRHAL_ADC_CHANNEL_8 },{ GPIOD, 1, STRHAL_GPIO_TYPE_OPP }, pyro2_cont, 1),
		pyro_igniter3(14,{ ADC1, STRHAL_ADC_CHANNEL_9 },{ GPIOD, 2, STRHAL_GPIO_TYPE_OPP }, pyro3_cont, 1),
		//pressure_control(20, press_1, solenoid_0, 1),
		//pressure_control(16, (GenericChannel&)*this, 1, solenoid_0, 1),

		//rocket(17, press_1, press_0, press_2, servo_0, servo_1, pyro_igniter0, pyro_igniter1, 1)
		speaker(STRHAL_TIM_TIM4, STRHAL_TIM_TIM4_CH2_PA12)
{
	registerChannel(&press_0);
	registerChannel(&press_1);
	registerChannel(&press_2);
	registerChannel(&press_3);
	registerChannel(&temp_0);
	registerChannel(&temp_1);
	registerChannel(&servo_0);
	registerChannel(&servo_1);
	registerChannel(&pyro0_cont);
	registerChannel(&pyro1_cont);
	registerChannel(&pyro2_cont);
	registerChannel(&pyro3_cont);
	registerChannel(&pyro_igniter0);
	registerChannel(&pyro_igniter1);
	registerChannel(&pyro_igniter2);
	registerChannel(&pyro_igniter3);
	//registerChannel(&pressure_control);
	//registerChannel(&rocket);

	registerModule(&flash);
}

int ECU_Lamarr::init()
{
	if (STRHAL_Init(STRHAL_SYSCLK_SRC_EXT, 8000000) != STRHAL_NOICE)
		return -1;

	// init status LEDs
	STRHAL_GPIO_SingleInit(&led_1, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&led_2, STRHAL_GPIO_TYPE_OPP);

	// init debug uart
	if (STRHAL_UART_Instance_Init(STRHAL_UART_DEBUG) != 0)
		return -1;

	if (can.init(receptor, heartbeatCan, COMMode::STANDARD_COM_MODE) != 0)
		return -1;

	if (GenericChannel::init() != 0)
		return -1;

	speaker.init();

	//STRHAL_GPIO_Write(&led_2, STRHAL_GPIO_VALUE_H);
	return 0;
}

int ECU_Lamarr::exec()
{
	STRHAL_OPAMP_Run();
	STRHAL_ADC_Run();
	STRHAL_QSPI_Run();

	if (can.exec() != 0)
		return -1;

	STRHAL_GPIO_Write(&led_1, STRHAL_GPIO_VALUE_H);
	STRHAL_UART_Debug_Write_Blocking("RUNNING\n", 8, 50);

	speaker.beep(6, 400, 300);

#ifdef UART_DEBUG
	STRHAL_UART_Listen(STRHAL_UART_DEBUG);

	uint8_t msgBuf[128] =
	{ 0 };
	uint8_t bufIndex = 0;
	bool msgStarted = false;
#endif
	while (1)
	{

		//detectReadoutMode();
#ifdef UART_DEBUG

		uint8_t tempBuf[64] =
		{ 0 };
		int32_t ret = STRHAL_UART_Read(STRHAL_UART_DEBUG, (char *) tempBuf, 64);
		if(ret > 0) {
			if(msgStarted) {
				memcpy(&msgBuf[bufIndex-1], tempBuf, ret);
				bufIndex += ret;
				if(tempBuf[ret-1] == 0x0A) { // msg ended
					msgStarted = false;
					bufIndex = 0;
					receptor((uint32_t) (msgBuf[0] << 11), &msgBuf[1], bufIndex - 1);
					memset(msgBuf, 0, 128);
				}
			} else {
				if(tempBuf[0] == 0x3A) { // start byte
					if(tempBuf[ret-1] == 0x0A) { // msg ended
						memcpy(msgBuf, tempBuf, ret - 1);
						receptor((uint32_t) (msgBuf[0] << 11), &msgBuf[1], ret - 1);
						memset(msgBuf, 0, 128);
					}
					bufIndex += ret;
					msgStarted = true;
					if(ret > 1)
						memcpy(msgBuf, &tempBuf[1], ret-1);
				}
				// else ignore msg
			}

		}
#endif
		if (GenericChannel::exec() != 0)
			return -1;

	}

	speaker.beep(6, 100, 100);

	return 0;
}

void ECU_Lamarr::testServo(ServoChannel &servo)
{
	servo.setTargetPos(0);
	servo.getPos();

	char buf[64];
	uint64_t t_last_sample = 0;
	uint8_t state = 0;
	while (1)
	{
		uint64_t t = STRHAL_Systick_GetTick();
		if ((t - t_last_sample) > 3000)
		{
			t_last_sample = t;
			if (state == 0)
			{
				servo.setTargetPos(63000);
				state = 1;
			}
			else
			{
				servo.setTargetPos(0);
				state = 0;
			}

		}
		sprintf(buf, "%d, %d, %d\n", servo.getCurrentMeasurement(), servo.getFeedbackMeasurement(), servo.getPos());
		STRHAL_UART_Debug_Write_DMA(buf, strlen(buf));
		if (GenericChannel::exec() != 0)
			return;
	}
}

void ECU_Lamarr::testChannels()
{
	char read[256], write[256];
	uint8_t state = 0;
	STRHAL_UART_Listen(STRHAL_UART_DEBUG);
	while (1)
	{
		int32_t n = STRHAL_UART_Read(STRHAL_UART_DEBUG, read, 2);
		if (n > 0)
		{
			AbstractChannel *channel = GenericChannel::channels[state];
			CHANNEL_TYPE type = channel->getChannelType();
			if (type == CHANNEL_TYPE_ADC16)
			{
				ADCChannel *adc = (ADCChannel*) channel;
				int nn = 0;
				while (nn == 0)
				{
					nn = STRHAL_UART_Read(STRHAL_UART_DEBUG, read, 2);
					std::sprintf(write, "ChannelId: %d, ChannelType: %d, Measurement: %d\n", channel->getChannelId(), type, adc->getMeasurement());
					STRHAL_UART_Debug_Write_Blocking(write, strlen(write), 50);
					STRHAL_Systick_BusyWait(500);
				}
			}
			else if (type == CHANNEL_TYPE_DIGITAL_OUT)
			{
				SetMsg_t set_msg =
				{ 0 };
				set_msg.variable_id = DIGITAL_OUT_STATE;
				set_msg.value = 1;
				uint8_t ret_n = 0;
				std::sprintf(write, "ChannelId: %d, ChannelType: %d\n", state, type);
				STRHAL_UART_Debug_Write_Blocking(write, strlen(write), 50);
				STRHAL_Systick_BusyWait(1000);
				STRHAL_UART_Debug_Write_Blocking("..Setting Output for 10s in\n", 28, 50);
				STRHAL_Systick_BusyWait(500);
				STRHAL_UART_Debug_Write_Blocking("..3s\n", 5, 50);
				STRHAL_Systick_BusyWait(1000);
				STRHAL_UART_Debug_Write_Blocking("..2s\n", 5, 50);
				STRHAL_Systick_BusyWait(1000);
				STRHAL_UART_Debug_Write_Blocking("..1s\n", 5, 50);
				STRHAL_Systick_BusyWait(1000);
				channel->processMessage(COMMON_REQ_SET_VARIABLE, (uint8_t*) &set_msg, ret_n);
				for (int i = 0; i < 5; i++)
				{
					uint8_t n = 0;
					uint8_t meas[2];
					channel->getSensorData(meas, n);
					std::sprintf(write, "...Output ON, Measurement: %d\n", meas[0] << 8 | meas[1]);
					STRHAL_UART_Debug_Write_Blocking(write, strlen(write), 50);
					STRHAL_Systick_BusyWait(2000);
				}
				set_msg.variable_id = DIGITAL_OUT_STATE;
				set_msg.value = 0;
				channel->processMessage(COMMON_REQ_SET_VARIABLE, (uint8_t*) &set_msg, ret_n);
				STRHAL_UART_Debug_Write_Blocking("..Output OFF\n", 13, 50);
			}
			else if (type == CHANNEL_TYPE_PNEUMATIC_VALVE)
			{
				std::sprintf(write, "Channel %d/type: %d not implemented\n", state, type);
				STRHAL_UART_Debug_Write_Blocking(write, strlen(write), 50);
				STRHAL_UART_Debug_Write_Blocking("Channel not implemented\n", 24, 50);
			}
			else if (type == CHANNEL_TYPE_SERVO)
			{
				std::sprintf(write, "Channel %d/type: %d not implemented\n", state, type);
				STRHAL_UART_Debug_Write_Blocking(write, strlen(write), 50);
			}
			else
			{
				std::sprintf(write, "Channel %d/type: %d not implemented\n", state, type);
				STRHAL_UART_Debug_Write_Blocking(write, strlen(write), 50);
			}
			state = (state == 20) ? 0 : (state + 1);
		}
		STRHAL_Systick_BusyWait(500);
	}
}
