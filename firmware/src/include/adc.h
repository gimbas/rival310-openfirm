#ifndef __ADC_H__
#define __ADC_H__

#include <stm32f10x.h>
#include <math.h>
#include "nvic.h"
#include "systick.h"

#define ADC_REFERENCE_VOLTAGE   2500.0
#define ADC_MAX_OUTPUT_CODE     ((1 << 12) - 1)
#define ADC_VOLTAGE_STEP        (ADC_REFERENCE_VOLTAGE / ADC_MAX_OUTPUT_CODE)

#define WATER_TEMP_SENSOR_1_ADC_CH  14
#define WATER_TEMP_SENSOR_2_ADC_CH  15
#define BAT_VOLT_ADC_CH             2

#define WATER_TEMP_SENSOR_FILTER_SIZE   2
#define BAT_VOLT_FILTER_SIZE            2

void adc_init();
void adc_wakeup();
void adc_sleep();
void adc_read_water_temp(double *pgData, uint32_t ulSamples);

#endif
