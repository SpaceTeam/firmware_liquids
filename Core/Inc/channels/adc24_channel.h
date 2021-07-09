#ifndef ADC24_CHANNEL_H_
#define ADC24_CHANNEL_H_

#include "adc24_channel_def.h"
#include "main.h"
#include "measurement.h"

typedef struct
{
	int32_t offset;
	int32_t thresholds[2];
} Adc24_Channel_t;

Result_t Adc24_ProcessMessage(uint8_t ch_id, uint8_t cmd_id, uint8_t *data, uint32_t length);
Result_t Adc24_GetData(uint8_t ch_id, int8_t *data, uint32_t *length);

#endif

