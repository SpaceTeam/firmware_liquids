#include "board_config.h"
#if BOARD == BLMB
#include "main.h"
#include "channels.h"
#include "speaker.h"
#include "can.h"
#include "foc/tmc6200/TMC6200_highLevel.h"
#include "foc/tmc6200/TMC6200.h"
#include "foc/tmc4671/TMC4671_highLevel.h"
#include "foc/tmc4671/TMC4671.h"
#include "foc/swdriver.h"
#include "foc/as5147.h"
#include "spi.h"
#include "serial.h"
#include "generic_channel.h"
#include "systick.h"
#include "ui.h"
#include "tim.h"
#include <string.h>

//@formatter:off
Node_t node = { .node_id = 0, .firmware_version = 0xDEADBEEF,
				.generic_channel = { 0 },
				.channels =
					{
							{ 0, CHANNEL_TYPE_SERVO, {{0}} }
					}
				};
//@formatter:on

void BLMB_InitAdc(void)
{
	Adc_Init();
	node.generic_channel.bus1_voltage = Adc_AddRegularChannel(PWR_BUS_VOLTAGE_1);
	node.generic_channel.bus2_voltage = Adc_AddRegularChannel(PWR_BUS_VOLTAGE_2);
	node.generic_channel.power_voltage = Adc_AddRegularChannel(SUPPLY_VOLTAGE_SENSE);
	node.generic_channel.power_current = Adc_AddRegularChannel(SUPPLY_CURRENT_SENSE);
	Adc_Calibrate();
	Adc_StartAdc();
}

void BLMB_InitFoc(void)
{

	TIM4_Init();

	LL_TIM_EnableCounter(TIM4);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);

	SPI1_Init(LL_SPI_DATAWIDTH_8BIT, LL_SPI_PHASE_2EDGE, LL_SPI_POLARITY_HIGH, LL_SPI_NSS_SOFT, LL_SPI_BAUDRATEPRESCALER_DIV128);

	Systick_BusyWait(100);
	tmc6200_highLevel_init();
	Systick_BusyWait(10);
	swdriver_setEnable(true);
	Systick_BusyWait(10);

	TMC4671_highLevel_init();
	Systick_BusyWait(10);

	TMC4671_highLevel_initEncoder();

	Systick_BusyWait(100)
	;

	tmc4671_writeInt(TMC4671_PID_POSITION_ACTUAL, (uint32_t) ((float) as5147_getAngle(BLMB_POSITION_ENCODER) * 32 * 231.1222));

	TMC4671_highLevel_setPosition(node.channels[BLMB_SERVO_CHANNEL].channel.servo.position_set);

	TMC4671_highLevel_positionMode2();

	Systick_BusyWait(100);

}

void BLMB_InitTIM(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	//TIM 5 change config mode timer
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV4;
	TIM_InitStruct.Prescaler = 63999; //250 Hz
	TIM_InitStruct.Autoreload = 499; //2 seconds

	LL_TIM_Init(TIM5, &TIM_InitStruct);

	NVIC_SetPriority(TIM5_IRQn, 3);
	NVIC_EnableIRQ(TIM5_IRQn);

	//TIM 6 input event timer
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV4;
	TIM_InitStruct.Prescaler = 15999; //1000 Hz
	TIM_InitStruct.Autoreload = 199; //200 milliseconds

	LL_TIM_Init(TIM6, &TIM_InitStruct);

	NVIC_SetPriority(TIM6_DAC_IRQn, 3);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void BLMB_InitGPIO(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	LL_EXTI_InitTypeDef EXTI_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP; //TODO: pull up?

	GPIO_InitStruct.Pin = BLMB_CW_Button_Pin;
	LL_GPIO_Init(BLMB_CW_Button_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BLMB_Sel_Button_Pin;
	LL_GPIO_Init(BLMB_Sel_Button_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BLMB_CCW_Button_Pin;
	LL_GPIO_Init(BLMB_CCW_Button_GPIO_Port, &GPIO_InitStruct);

	//EXTI
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE8);
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE9);
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE15);

	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_8 | LL_EXTI_LINE_9 | LL_EXTI_LINE_15;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.LineCommand = ENABLE;
	LL_EXTI_Init(&EXTI_InitStruct);


	//enable exti interrupts
	NVIC_SetPriority(EXTI9_5_IRQn, 3);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	NVIC_SetPriority(EXTI15_10_IRQn, 3);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void BLMB_main(void)
{
	uint64_t tick = 0;
	uint64_t old_tick = 0;

	BLMB_InitAdc();

	BLMB_InitFoc();

	BLMB_InitTIM();

	BLMB_InitGPIO();

	char serial_str[1000] =
	{ 0 };
	while (1)
	{
		tick = Systick_GetTick();
		Speaker_Update(tick);
		Can_checkFifo(LCB_MAIN_CAN_BUS);
		Can_checkFifo(1);

	//	TMC4671_highLevel_setPosition(node.channels[BLMB_SERVO_CHANNEL].channel.servo.position_set);
		if (tick - old_tick >= 1000)
		{
			old_tick = tick;
			Serial_PrintHex(tmc6200_readInt(TMC6200_GSTAT));
			Serial_PrintInt(tick);
		}
		if (Serial_CheckInput(serial_str))
		{
			Serial_PrintString(serial_str);
			node.channels[BLMB_SERVO_CHANNEL].channel.servo.position_set = atoi(serial_str);

		}

	}
}


//IRQHandlers
#define COOLDOWN_TICKS (100) //10 ms cooldown
volatile uint64_t cooldown_start_CW_button = 0;
volatile uint64_t cooldown_start_Sel_button = 0;
volatile uint64_t cooldown_start_CCW_button = 0;

typedef enum
{
	BLMB_NORMAL = 0,
	BLMB_CALIBRATE_OPEN = 1,
	BLMB_CALIBRATE_CLOSE = 2,
} blmb_config_state_t;

typedef enum
{
	BLMB_SPEED_SLOW = 0,
	BLMB_SPEED_MEDIUM = 1,
	BLMB_SPEED_FAST = 2,
} blmb_speed_state_t;

//volatile blmb_config_state_t blmb_CW_button_state = BLMB_RESET;
volatile blmb_config_state_t blmb_config_state = BLMB_NORMAL;
volatile blmb_speed_state_t blmb_speed_state = BLMB_SPEED_MEDIUM;
volatile uint8_t blmb_config_state_changed = 0;

void EXTI9_5_IRQHandler(void)
{
	uint64_t curr_ticks = Systick_GetTick();
	//CW button edge detected
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8))
	{
		//only
		if (((curr_ticks - cooldown_start_CW_button) > COOLDOWN_TICKS))
		{
			if (LL_GPIO_IsInputPinSet(BLMB_CW_Button_GPIO_Port, BLMB_CW_Button_Pin) == SET)
			{
				LL_TIM_EnableCounter(TIM6);
			}
			else
			{
				LL_TIM_DisableCounter(TIM6);
				LL_TIM_SetCounter(TIM6, 0);
			}
			cooldown_start_CW_button = curr_ticks;

			LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
		}


	}

	//Select button edge detected
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9))
	{
		//only
		if (((curr_ticks - cooldown_start_Sel_button) > COOLDOWN_TICKS))
		{
			if (LL_GPIO_IsInputPinSet(BLMB_Sel_Button_GPIO_Port, BLMB_Sel_Button_Pin) == SET)
			{
				LL_TIM_EnableCounter(TIM5);
			}
			else
			{
				LL_TIM_DisableCounter(TIM5);
				LL_TIM_SetCounter(TIM5, 0);
				if (!blmb_config_state_changed)
				{
					switch (blmb_speed_state)
					{
						case BLMB_SPEED_SLOW:
							blmb_speed_state = BLMB_SPEED_MEDIUM;
							Speaker_Set(659, 500, 0, 1);
							break;
						case BLMB_SPEED_MEDIUM:
							blmb_speed_state = BLMB_SPEED_FAST;
							Speaker_Set(659, 200, 50, 2);
							break;
						case BLMB_SPEED_FAST:
							blmb_speed_state = BLMB_SPEED_SLOW;
							Speaker_Set(659, 160, 40, 3);
							break;
						default:
							break;
					}
				}
				blmb_config_state_changed = 0;
			}
			cooldown_start_Sel_button = curr_ticks;

			LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
		}


	}
}

void EXTI15_10_IRQHandler(void)
{
	uint64_t curr_ticks = Systick_GetTick();

	//CCW button edge detected
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15))
	{
		//only
		if (((curr_ticks - cooldown_start_CCW_button) > COOLDOWN_TICKS))
		{
			if (LL_GPIO_IsInputPinSet(BLMB_CCW_Button_GPIO_Port, BLMB_CCW_Button_Pin) == SET)
			{
				LL_TIM_EnableCounter(TIM6);
			}
			else
			{
				LL_TIM_DisableCounter(TIM6);
				LL_TIM_SetCounter(TIM6, 0);
			}
			cooldown_start_CCW_button = curr_ticks;

			LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
		}


	}
}

void TIM5_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6))
	{
		LL_TIM_DisableCounter(TIM5);
		blmb_config_state_changed = 1;
		if (blmb_config_state == BLMB_NORMAL)
		{
			blmb_config_state = BLMB_CALIBRATE_OPEN;
			Speaker_Set(370, 1000, 0, 1);
		}
		else if (blmb_config_state == BLMB_CALIBRATE_OPEN)
		{
			blmb_config_state = BLMB_CALIBRATE_CLOSE;
			Speaker_Set(440, 300, 100, 2);

			//save open endpoint
		}
		else if (blmb_config_state == BLMB_CALIBRATE_CLOSE)
		{
			blmb_config_state = BLMB_NORMAL;
			Speaker_Set(440, 300, 100, 1);
			Speaker_Set(587, 500, 100, 1);

			//save close endpoint
		}

		LL_TIM_ClearFlag_UPDATE(TIM5);
	}
	else
	{
		Serial_PrintString("big oof, tim5 handler triggered but its not the update it");
	}
}

void TIM6_DAC_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6))
	{
		if (LL_GPIO_IsInputPinSet(BLMB_CW_Button_GPIO_Port, BLMB_CW_Button_Pin) == SET
				&& LL_GPIO_IsInputPinSet(BLMB_Sel_Button_GPIO_Port, BLMB_Sel_Button_Pin) == RESET
				&& LL_GPIO_IsInputPinSet(BLMB_CCW_Button_GPIO_Port, BLMB_CCW_Button_Pin) == RESET)
		{
			//set bl motor position cw with blmb_speed_state
			Speaker_Set(1000, 100, 100, 1);
		}
		else if (LL_GPIO_IsInputPinSet(BLMB_CW_Button_GPIO_Port, BLMB_CW_Button_Pin) == RESET
				&& LL_GPIO_IsInputPinSet(BLMB_Sel_Button_GPIO_Port, BLMB_Sel_Button_Pin) == RESET
				&& LL_GPIO_IsInputPinSet(BLMB_CCW_Button_GPIO_Port, BLMB_CCW_Button_Pin) == SET)
		{
			//set bl motor position ccw with blmb_speed_state
			Speaker_Set(1000, 100, 100, 2);
		}
		LL_TIM_ClearFlag_UPDATE(TIM5);
	}
	else
	{
		Serial_PrintString("big oof, tim6 handler triggered but its not the update it");
	}
}

#endif
