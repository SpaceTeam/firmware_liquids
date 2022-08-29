#include "../../Inc/Modules/VL53L1X.h"
#include <stm32g4xx_ll_exti.h>
#include <stm32g4xx_ll_system.h>

#include <cstring>
#include <cstdio>

VL53L1X::VL53L1X(const STRHAL_I2C_Id_t &i2cId, const STRHAL_GPIO_t &enablePin) :
		i2cId(i2cId), enablePin(enablePin)
{
}

int VL53L1X::init()
{
	if (STRHAL_I2C_Instance_Init(i2cId) < 0)
		return -1;

	bool ret = true;

	ret &= writeRegister8(SOFT_RESET, 0x00);
	LL_mDelay(100);
	ret &= writeRegister8(SOFT_RESET, 0x01);

	// give it some time to boot; otherwise the sensor NACKs during the readReg()
	// call below and the Arduino 101 doesn't seem to handle that well
	LL_mDelay(1);

	if (readRegister16(IDENTIFICATION__MODEL_ID) != 0xEACC) // return if sensor not connected
	{
		//STRHAL_UART_Debug_Write_Blocking("FAIL\n", 5, 100);
		return 0;
	}

	writeRegister8(PAD_I2C_HV__EXTSUP_CONFIG, readRegister8(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);

	// VL53L1_StaticInit() begin

	// VL53L1_set_preset_mode() begin

	// VL53L1_preset_mode_standard_ranging() begin

	// values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
	// (API uses these in VL53L1_init_tuning_parm_storage_struct())

	fast_osc_frequency = readRegister16(OSC_MEASURED__FAST_OSC__FREQUENCY);
	osc_calibrate_val = readRegister16(RESULT__OSC_CALIBRATE_VAL);

	// static config
	ret &= writeRegister16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
	ret &= writeRegister8(GPIO__TIO_HV_STATUS, 0x02);
	ret &= writeRegister8(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
	ret &= writeRegister8(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
	ret &= writeRegister8(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
	ret &= writeRegister8(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
	ret &= writeRegister8(ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
	ret &= writeRegister8(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

	// general config
	ret &= writeRegister16(SYSTEM__THRESH_RATE_HIGH, 0x0000);
	ret &= writeRegister16(SYSTEM__THRESH_RATE_LOW, 0x0000);
	ret &= writeRegister8(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

	// timing config
	// most of these settings will be determined later by distance and timing
	// budget configuration
	ret &= writeRegister16(RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
	ret &= writeRegister16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

	// dynamic config

	ret &= writeRegister8(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
	ret &= writeRegister8(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
	ret &= writeRegister8(SD_CONFIG__QUANTIFIER, 2); // tuning parm default

	// VL53L1_preset_mode_standard_ranging() end

	// from VL53L1_preset_mode_timed_ranging_*
	// GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
	// and things don't seem to work if we don't set GPH back to 0 (which the API
	// does here).
	ret &= writeRegister8(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
	ret &= writeRegister8(SYSTEM__SEED_CONFIG, 1); // tuning parm default

	// from VL53L1_config_low_power_auto_mode
	ret &= writeRegister8(SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
	ret &= writeRegister16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
	ret &= writeRegister8(DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

	// VL53L1_set_preset_mode() end

	// default to long range, 50 ms timing budget
	// note that this is different than what the API defaults to
	ret &= setDistanceMode();
	ret &= setMeasurementTimingBudget(10000);

	// VL53L1_StaticInit() end

	// the API triggers this change in VL53L1_init_and_start_range() once a
	// measurement is started; assumes MM1 and MM2 are disabled
	ret &= writeRegister16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, readRegister16(MM_CONFIG__OUTER_OFFSET_MM) * 4);

	//ret &= startContinuous(500);

	return 0;
}

int VL53L1X::exec()
{
	uint64_t time = STRHAL_Systick_GetTick();
	if ((time - timeLastSample) < EXEC_SAMPLE_TICKS)
		return 0;

	timeLastSample = time;

	if (read() != 0)
		timeoutCounter++;

	if (timeoutCounter > 50)
	{
		//STRHAL_UART_Debug_Write_Blocking("RESET\n", 6, 100);
		LL_I2C_DeInit(I2C3);
		//LL_I2C_Disable(I2C3);
		LL_mDelay(100);
		init();
		//STRHAL_I2C_Instance_Init(i2cId);
		LL_mDelay(10);
		//LL_I2C_Enable(I2C3);
		timeoutCounter = 0;
	}

	return 0;
}

int VL53L1X::reset()
{
	STRHAL_GPIO_Write(&enablePin, STRHAL_GPIO_VALUE_L);
	LL_mDelay(100);
	STRHAL_GPIO_Write(&enablePin, STRHAL_GPIO_VALUE_H);
	LL_mDelay(100);
	return 0;
}

bool VL53L1X::dataReady()
{
	uint8_t value;
	uint8_t data[] = { (uint8_t)(GPIO__TIO_HV_STATUS >> 8), (uint8_t)(GPIO__TIO_HV_STATUS) };
	if (STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, data, 2) != 2)
		return false;
	if (STRHAL_I2C_Receive(i2cId, (SLAVE_ADDR | 0x1), &value, 1) != 1)
		return false;

	return (value & 0x01) == 0;
}

// Write an 8-bit register
bool VL53L1X::writeRegister8(uint16_t reg, uint8_t value)
{
	uint8_t data[] = { (uint8_t)(reg >> 8), (uint8_t)(reg), value };
	uint8_t sent = STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, data, 3);

	return (sent == 3);
}

// Write a 16-bit register
bool VL53L1X::writeRegister16(uint16_t reg, uint16_t value)
{
	uint8_t data[] = { (uint8_t)(reg >> 8), (uint8_t)(reg), (uint8_t)(value >> 8), (uint8_t)(value) };
	uint8_t sent = STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, data, 4);

	return (sent == 4);
}

// Write a 32-bit register
bool VL53L1X::writeRegister32(uint16_t reg, uint32_t value)
{
	uint8_t data[] = { (uint8_t)(reg >> 8), (uint8_t)(reg), (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value) };
	uint8_t sent = STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, data, 6);

	return (sent == 6);
}

// Read an 8-bit register
uint8_t VL53L1X::readRegister8(uint16_t reg)
{
	uint8_t value;
	uint8_t data[] = { (uint8_t)(reg >> 8), (uint8_t)(reg) };
	STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, data, 2);
	STRHAL_I2C_Receive(i2cId, (SLAVE_ADDR | 0x1), &value, 1);

	return value;
}

// Read a 16-bit register
uint16_t VL53L1X::readRegister16(uint16_t reg)
{
	uint8_t value[2];
	uint8_t data[] = { (uint8_t)(reg >> 8), (uint8_t)(reg) };
	STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, data, 2);
	STRHAL_I2C_Receive(i2cId, (SLAVE_ADDR | 0x1), value, 2);

	return ( (value[0] << 8) | value[1] );
}

// Read a 32-bit register
uint32_t VL53L1X::readRegister32(uint16_t reg)
{
	uint8_t value[4];
	uint8_t data[] = { (uint8_t)(reg >> 8), (uint8_t)(reg) };
	STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, data, 2);
	STRHAL_I2C_Receive(i2cId, (SLAVE_ADDR | 0x1), value, 4);

	return ( (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3] );
}

bool VL53L1X::startContinuous(uint32_t period_ms)
{
	bool ret = true;
	// from VL53L1_set_inter_measurement_period_ms()
	ret &= writeRegister32(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);

	ret &= writeRegister8(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
	ret &= writeRegister8(SYSTEM__MODE_START, 0x40); // mode_range__timed

	return ret;
}

// Returns a range reading in millimeters when continuous mode is active. If
// blocking is true (the default), this function waits for a new measurement to
// be available. If blocking is false, it will try to return data immediately.
// (readSingle() also calls this function after starting a single-shot range
// measurement)
int VL53L1X::read()
{

	if(state == Ready)
	{
		(void) writeRegister8(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
		(void) writeRegister8(SYSTEM__MODE_START, 0x10); // mode_range__timed

		state = Polling;
	}

	if (dataReady())
	{
		uint8_t req[] = { (uint8_t)(RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 >> 8), (uint8_t)(RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0) };
		int32_t sent = STRHAL_I2C_Transmit(i2cId, SLAVE_ADDR, req, 2);
		if (sent != 2)
			return -1;

		uint8_t rec[15] =
		{ 0 };

		int32_t recv = STRHAL_I2C_Receive(i2cId, (SLAVE_ADDR | 0x1), rec, 15);
		if (recv != 15)
			return -1;

		uint16_t dss_actual_effective_spads_sd0  = (uint16_t) rec[0] << 8;
		dss_actual_effective_spads_sd0 |= rec[1];

		uint16_t ambient_count_rate_mcps_sd0  = (uint16_t)rec[4] << 8;
		ambient_count_rate_mcps_sd0 |= rec[5];

		uint16_t final_crosstalk_corrected_range_mm_sd0  = (uint16_t)rec[10] << 8;
		final_crosstalk_corrected_range_mm_sd0 |= rec[11];

		uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)rec[12] << 8;
		peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |= rec[13];

		uint16_t spadCount = dss_actual_effective_spads_sd0;

		if (spadCount != 0)
		{
			// "Calc total rate per spad"

			uint32_t totalRatePerSpad = (uint32_t) peak_signal_count_rate_crosstalk_corrected_mcps_sd0 + ambient_count_rate_mcps_sd0;

			// "clip to 16 bits"
			if (totalRatePerSpad > 0xFFFF)
			{
				totalRatePerSpad = 0xFFFF;
			}

			// "shift up to take advantage of 32 bits"
			totalRatePerSpad <<= 16;

			totalRatePerSpad /= spadCount;

			if (totalRatePerSpad != 0)
			{
				// "get the target rate and shift up by 16"
				uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

				// "clip to 16 bit"
				if (requiredSpads > 0xFFFF)
				{
					requiredSpads = 0xFFFF;
				}

				// "override DSS config"
				(void) writeRegister16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
				// DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS
			}
			else // divide by zero, set to mid point
			{
				(void) writeRegister16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
			}
		}
		else // divide by zero, set to mid point
		{
			(void) writeRegister16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
		}

		//getRangingData();
		uint16_t range = final_crosstalk_corrected_range_mm_sd0;

		// "apply correction gain"
		// gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
		// Basically, this appears to scale the result by 2011/2048, or about 98%
		// (with the 1024 added for proper rounding).
		measurement = (uint16_t) (((uint32_t)range * 2011 + 0x0400) / 0x0800);
		timeoutCounter = 0;
		(void) writeRegister8(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

		state = Ready;
		return 0;
	}
	return -1;
}

bool VL53L1X::setDistanceMode()
{
	bool ret = true;
    // timing config
	ret &= writeRegister8(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
	ret &= writeRegister8(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
	ret &= writeRegister8(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

    // dynamic config
	ret &= writeRegister8(SD_CONFIG__WOI_SD0, 0x07);
	ret &= writeRegister8(SD_CONFIG__WOI_SD1, 0x05);
	ret &= writeRegister8(SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
	ret &= writeRegister8(SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

	return 0;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate
// measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool VL53L1X::setMeasurementTimingBudget(uint32_t budget_us)
{
	bool ret = true;
  // assumes PresetMode is LOWPOWER_AUTONOMOUS
	uint32_t range_config_timeout_us = budget_us -= TimingGuard;
	if (range_config_timeout_us > 1100000)
		return false; // FDA_MAX_TIMING_BUDGET_US * 2

	range_config_timeout_us /= 2;

	// VL53L1_calc_timeout_register_values() begin

	uint32_t macro_period_us;

	// "Update Macro Period for Range A VCSEL Period"
	macro_period_us = calcMacroPeriod(readRegister8(RANGE_CONFIG__VCSEL_PERIOD_A));

	// "Update Phase timeout - uses Timing A"
	// Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
	// via VL53L1_get_preset_mode_timing_cfg().
	uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
	if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
	ret &= writeRegister8(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

	// "Update MM Timing A timeout"
	// Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
	// via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
	// actually ends up with a slightly different value because it gets assigned,
	// retrieved, recalculated with a different macro period, and reassigned,
	// but it probably doesn't matter because it seems like the MM ("mode
	// mitigation"?) sequence steps are disabled in low power auto mode anyway.
	ret &= writeRegister16(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
	timeoutMicrosecondsToMclks(1, macro_period_us)));

	// "Update Range Timing A timeout"
	ret &= writeRegister16(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
	timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

	// "Update Macro Period for Range B VCSEL Period"
	macro_period_us = calcMacroPeriod(readRegister8(RANGE_CONFIG__VCSEL_PERIOD_B));

	// "Update MM Timing B timeout"
	// (See earlier comment about MM Timing A timeout.)
	ret &= writeRegister16(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
	timeoutMicrosecondsToMclks(1, macro_period_us)));

	// "Update Range Timing B timeout"
	ret &= writeRegister16(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
	timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

	// VL53L1_calc_timeout_register_values() end

	return ret;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t VL53L1X::decodeTimeout(uint16_t reg_val)
{
	return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t VL53L1X::encodeTimeout(uint32_t timeout_mclks)
{
	// encoded format: "(LSByte * 2^MSByte) + 1"

	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0)
	{
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0)
		{
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);
	}
	else
	{
		return 0;
	}
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t VL53L1X::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
	return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t VL53L1X::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
	return (macro_period_us > 0) ? ((((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us) : 0;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t VL53L1X::calcMacroPeriod(uint8_t vcsel_period)
{
	// from VL53L1_calc_pll_period_us()
	// fast osc frequency in 4.12 format; PLL period in 0.24 format
	uint32_t pll_period_us = 0;
	if (fast_osc_frequency != 0)
		pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

	// from VL53L1_decode_vcsel_period()
	uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

	// VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
	uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
	macro_period_us >>= 6;
	macro_period_us *= vcsel_period_pclks;
	macro_period_us >>= 6;

	return macro_period_us;
}
