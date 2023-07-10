#include "FOC.hpp"

#include <cstdio>
#include <cstring>
#include <stdbool.h>
#include <stdlib.h>

#include "../../Drivers/STM32G474/MX_Gen/Inc/adc.h"
#include "../../Drivers/STM32G474/MX_Gen/Inc/cordic.h"
#include "../../Drivers/STM32G474/MX_Gen/Inc/dma.h"
#include "../../Drivers/STM32G474/MX_Gen/Inc/gpio.h"
#include "../../Drivers/STM32G474/MX_Gen/Inc/spi.h"
#include "../../Drivers/STM32G474/MX_Gen/Inc/tim.h"


volatile static bool current_cal_mode = false;

volatile static bool angle_m_fixed_mode = false;
volatile static uint16_t angle_m_fixed = 0;


volatile static uint16_t angle_m = 0;
volatile static uint32_t angle_e = 0;
volatile static float cos_angle_e = 0.0;
volatile static float sin_angle_e = 0.0;

volatile static int32_t Iu_offset = 0;
volatile static int32_t Iv_offset = 0;
volatile static int32_t Iw_offset = 0;

volatile static int32_t Id = 0;
volatile static int32_t Iq = 0;

volatile static int32_t Id_target = 0;
volatile static int32_t Iq_target = 0;

static PID pid_Id{CURRENT_CTRL_P, CURRENT_CTRL_I, 0, CURRENT_CTRL_RATE, VOLTAGE_LIMIT};
static PID pid_Iq{CURRENT_CTRL_P, CURRENT_CTRL_I, 0, CURRENT_CTRL_RATE, VOLTAGE_LIMIT};

volatile static int32_t Ud = 0;
volatile static int32_t Uq = 0;


volatile static int32_t velocity = 0;
volatile static int32_t velocity_target = 0;
volatile static bool velocity_control_active = false;
static PID pid_velocity{VELOCITY_CTRL_P, VELOCITY_CTRL_I, 0, VELOCITY_CTRL_RATE, CURRENT_LIMIT};


volatile static int32_t position = 0;
volatile static int32_t position_target = 0;
volatile static bool position_control_active = false;
static PID pid_position{POSITION_CTRL_P, POSITION_CTRL_I, 0, POSITION_CTRL_RATE, VELOCITY_LIMIT};


extern "C" {
	void ADC1_2_IRQHandler(void)
	{
		if(LL_ADC_IsActiveFlag_JEOS(ADC1))
		{
			LL_ADC_ClearFlag_JEOS(ADC1);
			LL_GPIO_ResetOutputPin(TP_1_GPIO_Port, TP_1_Pin);

			if(angle_m_fixed_mode) angle_m = angle_m_fixed;
			else angle_m = LL_TIM_GetCounter(TIM3) & 0xFFF; // 12bit

			angle_e = (((uint32_t)angle_m * POLE_PAIRS) << 4) & 0xFFFF; // q1.15

			LL_CORDIC_WriteData(CORDIC, (0x7FFF << 16) | angle_e); // modulo q1.15 = (almost) one, angle q1.15
			uint32_t cos_sin_angle_e = LL_CORDIC_ReadData(CORDIC); // 2x q1.15
			cos_angle_e = (float)((int16_t)(cos_sin_angle_e & 0xFFFF)) / 32768.0f; // q1.15 to float
			sin_angle_e = (float)((int16_t)((cos_sin_angle_e >> 16) & 0xFFFF)) / 32768.0f; // q1.15 to float

			// raw coil currents, mA
			int32_t Iu = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1) * CURRENT_FACTOR_mA_PER_LSB;
			int32_t Iv = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) * CURRENT_FACTOR_mA_PER_LSB;
			int32_t Iw = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC3, LL_ADC_INJ_RANK_1) * CURRENT_FACTOR_mA_PER_LSB;

	//		uint16_t temperature = LL_ADC_REG_ReadConversionData12(ADC3); // ToDo: read somewhere else?

			if(current_cal_mode) // current is zero ToDo: filtering
			{
				Iu_offset = Iu;
				Iv_offset = Iv;
				Iw_offset = Iw;
			}

			// coil currents, mA
			Iu -= Iu_offset;
			Iv -= Iv_offset;
			Iw -= Iw_offset;

			// check if sum close to zero //ToDo: improve accuracy
			if(labs(Iu + Iv + Iw) < 500) LL_GPIO_ResetOutputPin(LED_2_GPIO_Port, LED_2_Pin);
			else LL_GPIO_SetOutputPin(LED_2_GPIO_Port, LED_2_Pin); //ToDo: proper error handling

			// ToDo: everything fixed point

			// stator referenced currents, mA ToDo: simplified version using only Iu and Iv: https://de.wikipedia.org/wiki/Clarke-Transformation
			int32_t Ia = 0.66666f * Iu - 0.33333f * Iv - 0.33333f * Iw;
			int32_t Ib = 0.00000f * Iu + 0.57735f * Iv - 0.57735f * Iw; // sqrt(3)/3 = 0.57735

			// rotor referenced currents, mA
			Id =   Ia * cos_angle_e + Ib * sin_angle_e;
			Iq = - Ia * sin_angle_e + Ib * cos_angle_e;

			Ud = pid_Id.update(Id_target, Id);
			Uq = pid_Iq.update(Iq_target, Iq);

			int32_t supply_voltage = (int32_t)LL_ADC_INJ_ReadConversionData12(ADC4, LL_ADC_INJ_RANK_1) * VOLTAGE_FACTOR_mV_PER_LSB; // ToDo: read somewhere else and increase sample time?

			// scale to supply voltage -> duty cycle
			int32_t Dd = (Ud * PWM_PERIOD) / supply_voltage;
			int32_t Dq = (Uq * PWM_PERIOD) / supply_voltage;

			// stator referenced duty cycles
			int32_t Da = Dd * cos_angle_e - Dq * sin_angle_e;
			int32_t Db = Dd * sin_angle_e + Dq * cos_angle_e;

			// coil duty cycles
			int32_t Du =   1.0f * Da + 0.00000f * Db;
			int32_t Dv = - 0.5f * Da + 0.86603f * Db; // sqrt(3)/2 = 0.86603
			int32_t Dw = - 0.5f * Da - 0.86603f * Db; // sqrt(3)/2 = 0.86603

			// limit pwm to leave space for current adc measurement and limit current ToDo: proper circle limitation, remove this?
			if(Du < - PWM_AMPLITUDE_LIMIT) Du = - PWM_AMPLITUDE_LIMIT;
			else if(Du > PWM_AMPLITUDE_LIMIT) Du = PWM_AMPLITUDE_LIMIT;
			if(Dv < - PWM_AMPLITUDE_LIMIT) Dv = - PWM_AMPLITUDE_LIMIT;
			else if(Dv > PWM_AMPLITUDE_LIMIT) Dv = PWM_AMPLITUDE_LIMIT;
			if(Dw < - PWM_AMPLITUDE_LIMIT) Dw = - PWM_AMPLITUDE_LIMIT;
			else if(Dw > PWM_AMPLITUDE_LIMIT) Dw = PWM_AMPLITUDE_LIMIT;

			// set PWM duty cycle
			LL_TIM_OC_SetCompareCH1(TIM1, PWM_PERIOD / 2 + Du);
			LL_TIM_OC_SetCompareCH2(TIM1, PWM_PERIOD / 2 + Dv);
			LL_TIM_OC_SetCompareCH3(TIM1, PWM_PERIOD / 2 + Dw);

			LL_GPIO_ResetOutputPin(TP_2_GPIO_Port, TP_2_Pin);
		}
	}


	void TIM1_CC_IRQHandler(void)
	{
		if(LL_TIM_IsActiveFlag_CC4(TIM1))
		{
			LL_TIM_ClearFlag_CC4(TIM1);
			LL_GPIO_SetOutputPin(TP_1_GPIO_Port, TP_1_Pin);
			LL_GPIO_SetOutputPin(TP_2_GPIO_Port, TP_2_Pin);
		}
	}


	void TIM8_CC_IRQHandler(void)
	{
		if(LL_TIM_IsActiveFlag_CC1(TIM8))
		{
			LL_TIM_ClearFlag_CC1(TIM8);
			LL_GPIO_ResetOutputPin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin);
		}
		if(LL_TIM_IsActiveFlag_CC2(TIM8))
		{
			LL_TIM_ClearFlag_CC2(TIM8);
			LL_SPI_TransmitData16(SPI3, 0x7FFC);
		}
	}


	void SPI3_IRQHandler(void)
	{
		if(LL_SPI_IsActiveFlag_RXNE(SPI3))
		{
			LL_GPIO_SetOutputPin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin);
			int16_t vel = (int16_t) LL_SPI_ReceiveData16(SPI3);
			vel &= 0x3FFF;
			vel |= ((vel & 0x2000) << 1) | ((vel & 0x2000) << 2);
#if ENCODER_INVERTED
			velocity = - vel * VELOCITY_FACTOR_RPM_PER_LSB;
#else
			velocity = vel * VELOCITY_FACTOR_RPM_PER_LSB;
#endif

			if(velocity_control_active) Iq_target = pid_velocity.update(velocity_target, velocity);
			else pid_velocity.reset();

			// position controller
			static uint8_t positionControlCnt = 0;
			if(++positionControlCnt > POS_CTRL_DIV)
			{
				positionControlCnt = 0;

				position = LL_TIM_GetCounter(TIM3) - 32768;
				if(position_control_active) velocity_target = pid_position.update(position_target, position);
				else pid_position.reset();
			}
		}
	}
}


FOC::FOC()
{

}


void FOC::init(volatile uint16_t* additionalAdcValues)
{
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_ADC4_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
	MX_CORDIC_Init();


	/* Gate Driver */

	LL_GPIO_SetOutputPin(EN_GPIO_Port, EN_Pin);

	LL_SPI_Enable(SPI1);
	LL_mDelay(10);

	uint16_t conf = DRV_SPI_FRAME(DRV_SPI_WRITE, 0x03, (((GATE_DRIVE_CURRENT_HS_RISING & 0xF) << 4) | (GATE_DRIVE_CURRENT_HS_FALLING & 0xF)));

	LL_GPIO_ResetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
	LL_mDelay(1);
	LL_SPI_TransmitData16(SPI1, conf);
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_mDelay(1);
	LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);

	LL_mDelay(10);

	conf = DRV_SPI_FRAME(DRV_SPI_WRITE, 0x04, ((0x7 << 8) | ((GATE_DRIVE_CURRENT_LS_RISING & 0xF) << 4) | (GATE_DRIVE_CURRENT_LS_FALLING & 0xF)));

	LL_GPIO_ResetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
	LL_mDelay(1);
	LL_SPI_TransmitData16(SPI1, conf);
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_mDelay(1);
	LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);

	LL_mDelay(10);

	conf = DRV_SPI_FRAME(DRV_SPI_WRITE, 0x06, 0x2c3); // gain=40, else standard

	LL_GPIO_ResetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
	LL_mDelay(1);
	LL_SPI_TransmitData16(SPI1, conf);
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_mDelay(1);
	LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);

	LL_mDelay(10);


	/* Motor PWM */
	DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP; // stop TIM1 on hitting a debug breakpoint --> pwm off

	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableIT_CC4(TIM1); // ToDo: debugging, remove

	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);

	//TODO: TIM1 BRK + COMP (I + U, I signed?) + DAC


	/* Encoder */

	LL_SPI_Enable(SPI3);

	//FIXME not working. if working, remove the direction change in TIM3 config and encoder velocity read
//	if(ENCODER_INVERTED)
//	{
//		LL_mDelay(1);
//		LL_GPIO_ResetOutputPin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin);
//		LL_mDelay(1);
//		LL_SPI_TransmitData16(SPI3, 0x0019); // write to SETTINGS2 register
//		while(LL_SPI_IsActiveFlag_BSY(SPI3));
//		LL_GPIO_SetOutputPin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin);
//
//		LL_mDelay(1);
//		LL_GPIO_ResetOutputPin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin);
//		LL_mDelay(1);
//		LL_SPI_TransmitData16(SPI3, (1 << 2)); // set DIR bit
//		while(LL_SPI_IsActiveFlag_BSY(SPI3));
//		LL_GPIO_SetOutputPin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin);
//	}

	LL_TIM_EnableCounter(TIM3);

	LL_SPI_EnableIT_RXNE(SPI3);

	LL_TIM_EnableCounter(TIM8);
	LL_TIM_EnableIT_CC1(TIM8);
	LL_TIM_EnableIT_CC2(TIM8);


	/* CORDIC */
	LL_CORDIC_Config(CORDIC,
	LL_CORDIC_FUNCTION_COSINE, // cosine function
	LL_CORDIC_PRECISION_6CYCLES, // max precision for q1.31 cosine ToDo: can make less for q1.15?
	LL_CORDIC_SCALE_0, // no scale
	LL_CORDIC_NBWRITE_1, // One input data: angle. Second input data (modulus) is 1 after cordic reset
	LL_CORDIC_NBREAD_1, // Two output data: cosine, then sine
	LL_CORDIC_INSIZE_16BITS, // q1.15 format for input data
	LL_CORDIC_OUTSIZE_16BITS); // q1.15 format for output data


	/* ADC */

	// external pressure & temp
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)additionalAdcValues);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 6);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC1));
	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC2));
	LL_ADC_StartCalibration(ADC3, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC3));
	LL_ADC_StartCalibration(ADC4, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC4));

	LL_mDelay(1);

	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);
	LL_ADC_Enable(ADC3);
	LL_ADC_Enable(ADC4);

	LL_mDelay(1);


	/* temp & pressure ADC conversions */
	LL_ADC_REG_StartConversion(ADC3); // power stage temp

	LL_ADC_REG_StartConversion(ADC2); // external pressure & temp


	/* Current & voltage ADC conversions */

	LL_ADC_EnableIT_JEOS(ADC1); // used to trigger FOC control loop

	// arm Inj sequences to be triggered by TIM1
	LL_ADC_INJ_StartConversion(ADC1);
	LL_ADC_INJ_StartConversion(ADC2);
	LL_ADC_INJ_StartConversion(ADC3);
	LL_ADC_INJ_StartConversion(ADC4);


	/* brake pwm */
//	  LL_TIM_EnableCounter(TIM2);
//	  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
//	  LL_TIM_OC_SetCompareCH3(TIM2, 300);


	/* calibrate current measurement */

	LL_mDelay(10);
	LL_GPIO_SetOutputPin(CAL_GPIO_Port, CAL_Pin);
	LL_mDelay(10);
	LL_GPIO_ResetOutputPin(CAL_GPIO_Port, CAL_Pin);
	LL_mDelay(10);
	current_cal_mode = true;
	LL_mDelay(10);
	current_cal_mode = false;

	/* enable motor PWM & encoder zeroing */

	angle_m_fixed_mode = true;
	angle_m_fixed = 0;

	LL_TIM_EnableAllOutputs(TIM1);

	// force rotor to electrical zero position
	Id_target = 5000;
	Iq_target = 0;

	LL_mDelay(1000); // let rotor settle

	LL_TIM_SetCounter(TIM3, 32768); // reset encoder position

	// remove current, switch to normal operation
	Id_target = 0;
	Iq_target = 0;
	LL_mDelay(100);
	angle_m_fixed_mode = false;
}


void FOC::exec()
{
	if(LL_GPIO_IsInputPinSet(FAULT_GPIO_Port, FAULT_Pin))
	{
		LL_GPIO_ResetOutputPin(LED_2_GPIO_Port, LED_2_Pin);
	}
	else
	{
		LL_GPIO_SetOutputPin(LED_2_GPIO_Port, LED_2_Pin);
		// ToDo proper error handling
	}
}


void FOC::enable_velocity_control(bool enabled)
{
	velocity_control_active = enabled;
}

void FOC::enable_position_control(bool enabled)
{
	position_control_active = enabled;
}

void FOC::enable_control(bool enabled)
{
	position_control_active = enabled;
	velocity_control_active = enabled;
	// reset current
	Id_target = 0;
	Iq_target = 0;
}

void FOC::set_velocity_target(int32_t vel_target)
{
	velocity_target = vel_target;
}

void FOC::set_position_target(int32_t pos_target)
{
	position_target = pos_target;
}

int32_t FOC::get_position()
{
	return position;
}

int32_t FOC::get_Iq()
{
	return Iq;
}
