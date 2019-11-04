#ifndef __ADC_H__
#define __ADC_H__

#include <stm32f10x.h>
#include <math.h>
#include "nvic.h"
#include "systick.h"

#define ADC_REFERENCE_VOLTAGE   2500.0
#define ADC_MAX_OUTPUT_CODE     ((1 << 12) - 1)
#define ADC_VOLTAGE_STEP        (ADC_REFERENCE_VOLTAGE / ADC_MAX_OUTPUT_CODE)

#define BAT_VOLT_DIV            2.0 // Voltage divider ratio
#define LOW_BAT_VOLTAGE         3400.0
#define LOW_BAT_VOLTAGE_HYST    100.0

#define LMT86_KA    -0.00347
#define LMT86_KB    -10.888
#define LMT86_KC    1777.3
#define LMT87_KA    -0.00433
#define LMT87_KB    -13.582
#define LMT87_KC    2230.8
#define LMT8X_TRANSFER_FUNC(a, b, c, mv) (((-(b) - sqrt((b) * (b) - 4.0 * (a) * ((c) - (mv)))) / 2.0 / (a)) + 30.0)

#define WATER_TEMP_SENSOR_1_ADC_CH  14
#define WATER_TEMP_SENSOR_2_ADC_CH  15
#define BAT_VOLT_ADC_CH             2

#define WATER_TEMP_SENSOR_FILTER_SIZE   2
#define BAT_VOLT_FILTER_SIZE            2

extern volatile uint8_t g_ubBatteryLow;

void adc_init();
void adc_wakeup();
void adc_sleep();
void adc_read_water_temp(double *pgData, uint32_t ulSamples);
void adc_read_battery(double *pgData, uint32_t ulSamples);

#endif
