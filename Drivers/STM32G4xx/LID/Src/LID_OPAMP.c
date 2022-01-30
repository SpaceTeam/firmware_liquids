#include "../Inc/LID_OPAMP.h"

void LID_OPAMP_Init() {

	// GPIO init
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);

	LL_GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_2;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// OPAMP INIT
	LL_OPAMP_InitTypeDef OPAMP_InitStruct =
	{ 0 };

	OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_HIGHSPEED;
	OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA;
	OPAMP_InitStruct.InputInverting = LL_OPAMP_INPUT_INVERT_IO0;
	OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO0;
	LL_OPAMP_Init(OPAMP2, &OPAMP_InitStruct);
	LL_OPAMP_Init(OPAMP3, &OPAMP_InitStruct);

	LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_32_OR_MINUS_31);
	LL_OPAMP_SetMode(OPAMP2, LL_OPAMP_MODE_FUNCTIONAL);
	LL_OPAMP_SetInternalOutput(OPAMP2, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
	LL_OPAMP_SetTrimmingMode(OPAMP2, LL_OPAMP_TRIMMING_FACTORY);

	LL_OPAMP_SetPGAGain(OPAMP3, LL_OPAMP_PGA_GAIN_32_OR_MINUS_31);
	LL_OPAMP_SetMode(OPAMP3, LL_OPAMP_MODE_FUNCTIONAL);
	LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
	LL_OPAMP_SetTrimmingMode(OPAMP3, LL_OPAMP_TRIMMING_FACTORY);
}

void LID_OPAMP_Run() {
	LL_OPAMP_Enable(OPAMP2);
	while(!LL_OPAMP_IsEnabled(OPAMP2))
		;
	LL_OPAMP_Enable(OPAMP3);
	while(!LL_OPAMP_IsEnabled(OPAMP3))
		;
}