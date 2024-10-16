#include <IOBv3.h>
#include <cstdio>
#include <cstring>

#if defined(IOBv3_BOARD)

IOBv3::IOBv3(uint32_t node_id, uint32_t fw_version, uint32_t refresh_divider) :
		GenericChannel(node_id, fw_version, refresh_divider),
		led1({ GPIOB, 15, STRHAL_GPIO_TYPE_OPP }),
		led2({ GPIOB, 14, STRHAL_GPIO_TYPE_OPP }),
		led_debug({ GPIOB, 11, STRHAL_GPIO_TYPE_OPP }),
		channel0(0,{ ADC5, STRHAL_ADC_CHANNEL_1 },		{ GPIOA,  9, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel1(1,{ ADC5, STRHAL_ADC_CHANNEL_13 },		{ GPIOC,  9, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel2(2,{ ADC2, STRHAL_ADC_CHANNEL_12 },		{ GPIOD,  8, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel3(3,{ ADC2, STRHAL_ADC_CHANNEL_13 },		{ GPIOA, 10, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel4(4,{ ADC3, STRHAL_ADC_CHANNEL_7 },		{ GPIOC,  6, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel5(5,{ ADC3, STRHAL_ADC_CHANNEL_4 },		{ GPIOA,  6, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel6(6,{ ADC1, STRHAL_ADC_CHANNEL_1 },		{ GPIOD,  2, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel7(7,{ ADC1, STRHAL_ADC_CHANNEL_8 },		{ GPIOC,  0, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel8(8,{ ADC2, STRHAL_ADC_CHANNEL_5 },		{ GPIOA,  4, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel9(9,{ ADC1, STRHAL_ADC_CHANNEL_4 },		{ GPIOD,  1, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel10(10,{ ADC1, STRHAL_ADC_CHANNEL_2 },	{ GPIOC,  3, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_REGULAR, 1),
		channel11(11,{ ADC2, STRHAL_ADC_CHANNEL_11 },	{ GPIOC,  5, STRHAL_GPIO_TYPE_OPP }, STRHAL_ADC_INTYPE_OPAMP, 1),

		//sense_5V(0, {ADC2, STRHAL_ADC_CHANNEL_5}, 1),
		speaker(STRHAL_TIM_TIM4, STRHAL_TIM_TIM4_CH2_PB7)
{
	registerChannel(&channel0);
	registerChannel(&channel1);
	registerChannel(&channel2);
	registerChannel(&channel3);
	registerChannel(&channel4);
	registerChannel(&channel5);
	registerChannel(&channel6);
	registerChannel(&channel7);
	registerChannel(&channel8);
	registerChannel(&channel9);
	registerChannel(&channel10);
	registerChannel(&channel11);

	registerModule(&flash);

}

int IOBv3::init()
{
	if (STRHAL_Init(STRHAL_SYSCLK_SRC_EXT, 8000000) != STRHAL_NOICE)
		return -1;

	// init status LEDs
	STRHAL_GPIO_SingleInit(&led1, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&led2, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&led_debug, STRHAL_GPIO_TYPE_OPP);




	// init debug uart
	if (STRHAL_UART_Instance_Init(STRHAL_UART_DEBUG) != 0)
		return -1;

	if (can.init(receptor, heartbeatCan, COMMode::STANDARD_COM_MODE) != 0)
		return -1;

	if (GenericChannel::init() != 0)
		return -1;

	speaker.init();
	STRHAL_GPIO_Write(&led_debug, STRHAL_GPIO_VALUE_H);

	STRHAL_UART_Debug_Write_Blocking("Started\n", 8, 50);


	return 0;
}

int IOBv3::exec()
{
	//STRHAL_OPAMP_Run();
	STRHAL_ADC_Run();
	STRHAL_QSPI_Run();

	if (can.exec() != 0)
		return -1;

	STRHAL_UART_Debug_Write_Blocking("RUNNING\n", 8, 50);

	speaker.beep(3, 300, 200);

	while (1)
	{
		if (GenericChannel::exec() != 0)
			return -1;
	}

	speaker.beep(6, 100, 100);

	return 0;
}
#endif
