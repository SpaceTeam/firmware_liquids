#include "../Inc/RCUv2.h"

#include <cstdio>
#include <cstring>
#include <stm32g4xx_ll_usart.h>

RCUv2::RCUv2(uint32_t node_id, uint32_t fw_version, uint32_t refresh_divider) :
		GenericChannel(node_id, fw_version, refresh_divider),
		led1({ GPIOD, 1, STRHAL_GPIO_TYPE_OPP }),
		led2({ GPIOD, 2, STRHAL_GPIO_TYPE_OPP }),
		baro(STRHAL_SPI_SPI1,{ STRHAL_SPI_SPI1_SCK_PA5, STRHAL_SPI_SPI1_MISO_PA6, STRHAL_SPI_SPI1_MOSI_PA7, STRHAL_SPI_SPI1_NSS_PA4, STRHAL_SPI_MODE_MASTER, STRHAL_SPI_CPOL_CPHASE_HH, 0x7, 0 },{ GPIOA, 3, STRHAL_GPIO_TYPE_IHZ }),
		imu(STRHAL_SPI_SPI3,{ STRHAL_SPI_SPI3_SCK_PC10, STRHAL_SPI_SPI3_MISO_PC11, STRHAL_SPI_SPI3_MOSI_PC12, STRHAL_SPI_SPI3_NSS_PA15, STRHAL_SPI_MODE_MASTER, STRHAL_SPI_CPOL_CPHASE_HH, 0x7, 0 },{ GPIOD, 0, STRHAL_GPIO_TYPE_IHZ }, 0x12),
		lora(STRHAL_SPI_SPI2,{ STRHAL_SPI_SPI2_SCK_PB13, STRHAL_SPI_SPI2_MISO_PB14, STRHAL_SPI_SPI2_MOSI_PB15, STRHAL_SPI_SPI2_NSS_PB12, STRHAL_SPI_MODE_MASTER, STRHAL_SPI_CPOL_CPHASE_LL, 0x7, 0 },{ GPIOC, 1, STRHAL_GPIO_TYPE_IHZ },{ GPIOC, 3, STRHAL_GPIO_TYPE_IHZ },{ GPIOB, 11, STRHAL_GPIO_TYPE_IHZ }),
		gnss(STRHAL_UART1,{ GPIOC, 7, STRHAL_GPIO_TYPE_OPP }),
		sense_5V(RCUv2_SENSE_5V,{ ADC1, STRHAL_ADC_CHANNEL_2 }, 1),
		sense_12V(RCUv2_SENSE_12V,{ ADC1, STRHAL_ADC_CHANNEL_3 }, 1),
		baro_channel(RCUv2_BARO, &baro, 1),
		x_accel(RCUv2_ACCEL_X, &imu, IMUMeasurement::X_ACCEL, 1),
		y_accel(RCUv2_ACCEL_Y, &imu, IMUMeasurement::Y_ACCEL, 1),
		z_accel(RCUv2_ACCEL_Z, &imu, IMUMeasurement::Z_ACCEL, 1),
		x_gyro(RCUv2_GYRO_X, &imu, IMUMeasurement::X_GYRO, 1),
		y_gyro(RCUv2_GYRO_Y, &imu, IMUMeasurement::Y_GYRO, 1),
		z_gyro(RCUv2_GYRO_Z, &imu, IMUMeasurement::Z_GYRO, 1),
		gps_longitude(RCUv2_GNSS_LONG, &gnss.gnssData.longitude, 1),
		gps_latitude(RCUv2_GNSS_LAT, &gnss.gnssData.latitude, 1),
		gps_altitude(RCUv2_GNSS_ALT, &gnss.gnssData.altitude, 1),
		gps_status(RCUv2_GNSS_STATUS, &gnss.gnssData.status, 1),
		out0(RCUv2_OUT0, { GPIOA, 0, STRHAL_GPIO_TYPE_IHZ }, 1),
		out1(RCUv2_OUT1, { GPIOC, 2, STRHAL_GPIO_TYPE_IHZ }, 1),
		out2(RCUv2_OUT2, { GPIOC, 0, STRHAL_GPIO_TYPE_IHZ }, 1),
		out3(RCUv2_OUT3, { GPIOC, 10, STRHAL_GPIO_TYPE_IHZ }, 1),
		radio(Radio::instance(node_id, lora)),
		speaker(STRHAL_TIM_TIM2, STRHAL_TIM_TIM2_CH3_PB10)
{
	// set pointer to radio object for static callbacks, enable Lora
	//GenericChannel::radioPtr = &radio; <- this might cause hardfault later on
	setLoraActive(true); // has to be enabled by request

	registerChannel(&sense_5V);
	registerChannel(&sense_12V);
	registerChannel(&baro_channel);
	registerChannel(&x_accel);
	registerChannel(&y_accel);
	registerChannel(&z_accel);
	registerChannel(&x_gyro);
	registerChannel(&y_gyro);
	registerChannel(&z_gyro);
	registerChannel(&gps_longitude);
	registerChannel(&gps_latitude);
	registerChannel(&gps_altitude);
	registerChannel(&gps_status);
	registerChannel(&out0);
	registerChannel(&out1);
	registerChannel(&out2);
	registerChannel(&out3);

	registerModule(&flash);
	registerModule(&gnss);
	registerModule(&baro);
	registerModule(&imu);

}

int RCUv2::init()
{
	if (STRHAL_Init(STRHAL_SYSCLK_SRC_EXT, 8000000) != STRHAL_NOICE)
		return -1;

	// init status LEDs
	STRHAL_GPIO_SingleInit(&led1, STRHAL_GPIO_TYPE_OPP);
	STRHAL_GPIO_SingleInit(&led2, STRHAL_GPIO_TYPE_OPP);

	// init debug uart
	if (STRHAL_UART_Instance_Init(STRHAL_UART_DEBUG) != 0)
		return -1;

	if (lora.init() != 0)
		return -1;

	if (can.init(receptorLora, heartbeatCan, COMMode::LISTENER_COM_MODE) != 0)
		return -1;

	if (radio.init(nullptr, heartbeatLora) != 0)
		return -1;

	if (GenericChannel::init() != 0)
		return -1;

	speaker.init();

	STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
	return 0;
}

int RCUv2::exec()
{
	//STRHAL_OPAMP_Run();
	STRHAL_ADC_Run();
	STRHAL_QSPI_Run();

	if (can.exec() != 0)
		return -1;

	if (radio.exec() != 0)
		return -1;

	STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
	STRHAL_UART_Debug_Write_Blocking("RUNNING\n", 8, 50);

	speaker.beep(2, 400, 300);

	LL_mDelay(2000);

	STRHAL_UART_Listen(STRHAL_UART_DEBUG);

#ifdef UART_DEBUG
	STRHAL_UART_Listen(STRHAL_UART_DEBUG);

	uint8_t msgBuf[128] =
	{ 0 };
	uint8_t bufIndex = 0;
	bool msgStarted = false;
#endif
	uint64_t prevTime = STRHAL_Systick_GetTick();
	while (1)
	{
		detectReadoutMode();
		testIMU();
#ifdef UART_DEBUG

		uint8_t tempBuf[64] =
		{ 0 };
		int32_t ret = STRHAL_UART_Read(STRHAL_UART_DEBUG, (char *) tempBuf, 64);
		if(ret > 0)
		{
			if(msgStarted)
			{
				memcpy(&msgBuf[bufIndex-1], tempBuf, ret);
				bufIndex += ret;
				if(tempBuf[ret-1] == 0x0A) // msg ended
				{
					msgStarted = false;
					bufIndex = 0;
					receptor((uint32_t) (msgBuf[0] << 11), &msgBuf[1], bufIndex - 1);
					memset(msgBuf, 0, 128);
				}
			}
			else
			{
				if(tempBuf[0] == 0x3A) // start byte
				{
					if(tempBuf[ret-1] == 0x0A) // msg ended
					{
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
		if(STRHAL_Systick_GetTick() - prevTime > 1000)
		{
			prevTime = STRHAL_Systick_GetTick();
			/*
			char buf[64] = { 0 };
			uint16_t measurement = 0;
			imu.getMeasurement(measurement, IMUMeasurement::X_ACCEL);
			sprintf(buf, "%d\n", measurement);
			STRHAL_UART_Debug_Write_Blocking(buf, strlen(buf), 100);
			*/
			/*
			SetMsg_t setMsg =
			{ 0 };
			setMsg.variable_id = 1; // servo target position
			setMsg.value = 1; // open servo
			can.sendAsMaster(8, 1, 4, (uint8_t*) &setMsg, 5 + sizeof(uint32_t));
			*/
		}

		if (GenericChannel::exec() != 0)
			return -1;
	}

	speaker.beep(6, 100, 100);

	return 0;
}

void RCUv2::testIMU()
{
	char buf[64] = { 0 };

	imu.read();

	uint16_t x_accel_measurement = 0;
	uint16_t y_accel_measurement = 0;
	uint16_t z_accel_measurement = 0;
	imu.getMeasurement(x_accel_measurement, IMUMeasurement::X_ACCEL);
	imu.getMeasurement(y_accel_measurement, IMUMeasurement::Y_ACCEL);
	imu.getMeasurement(z_accel_measurement, IMUMeasurement::Z_ACCEL);
	sprintf(buf, "x: %d, y: %d, z: %d\n", x_accel_measurement, y_accel_measurement, z_accel_measurement);
	STRHAL_UART_Debug_Write_Blocking(buf, strlen(buf), 100);
	double x_accel = (double)(((int16_t)x_accel_measurement) * 16.0 / 32768.0);
	if( x_accel * x_accel > 0.25){
		speaker.beep(1, 100, 500);
	}

	double y_accel = (double)(((int16_t)y_accel_measurement) * 16.0 / 32768.0);
	if( y_accel * y_accel > 0.25){
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_H);
	}else{
		STRHAL_GPIO_Write(&led2, STRHAL_GPIO_VALUE_L);
	}

	double z_accel = (double)(((int16_t)z_accel_measurement) * 16.0 / 32768.0);
	if( z_accel * z_accel > 0.25){
		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_H);
	}else{
		STRHAL_GPIO_Write(&led1, STRHAL_GPIO_VALUE_L);
	}
}
void RCUv2::testGNSS()
{
	char buf[64] = { 0 };
	int32_t lon = gnss.gnssData.longitude;
	int32_t lat = gnss.gnssData.latitude;
	int32_t alt = gnss.gnssData.altitude;

	int32_t status = gnss.gnssData.status;

	sprintf(buf, "hello world");
}
