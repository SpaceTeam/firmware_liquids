#pragma once

#include "PID.hpp"


#define POLE_PAIRS 14


//ToDo increase?
#define VELOCITY_LIMIT 600 // RPM
#define CURRENT_LIMIT 10000 // mA
#define VOLTAGE_LIMIT 30000 // mV
#define PWM_AMPLITUDE_LIMIT 1000


#define PWM_PERIOD 4096


#define CURRENT_CTRL_RATE (160000000 / (PWM_PERIOD - 1) / 2) //ToDo: Is this correct?
#define CURRENT_CTRL_P 0.05
#define CURRENT_CTRL_I 500.0


#define VELOCITY_CTRL_RATE 2000
#define VELOCITY_CTRL_P 20.0
#define VELOCITY_CTRL_I 500.0


#define POS_CTRL_DIV 2
#define POSITION_CTRL_RATE (VELOCITY_CTRL_RATE / POS_CTRL_DIV)
#define POSITION_CTRL_P 0.5
#define POSITION_CTRL_I 0.0


#define CURRENT_FACTOR_mA_PER_LSB 20 // 20mA/lsb with gain 40
#define VOLTAGE_FACTOR_mV_PER_LSB 6.85
#define VELOCITY_FACTOR_RPM_PER_LSB 4.0235


#define GATE_DRIVE_CURRENT_HS_RISING 7
#define GATE_DRIVE_CURRENT_HS_FALLING 12
#define GATE_DRIVE_CURRENT_LS_RISING 7
#define GATE_DRIVE_CURRENT_LS_FALLING 12


#define ENCODER_RESOLUTION 4096
#define ENCODER_INVERTED true


#define DRV_SPI_FRAME(RW, ADDR, DATA) (((RW & 0x1) << 15) | ((ADDR & 0xF) << 11) | (DATA & 0x7FF))
#define DRV_SPI_READ 1
#define DRV_SPI_WRITE 0


class FOC
{
	public:
		FOC();

		void init(volatile uint16_t* additionalAdcValues);
		void exec();

		void enable_velocity_control(bool enabled);
		void enable_position_control(bool enabled);
		void enable_control(bool enabled);

		void set_velocity_target(int32_t pos_target);
		void set_position_target(int32_t pos_target);

		int32_t get_position();
		int32_t get_Iq();

	private:
};
