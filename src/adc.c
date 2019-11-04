#include "adc.h"

volatile uint8_t g_ubBatteryLow = 0;

static volatile uint8_t ubADC1DMAComplete = 0;
static volatile uint16_t pusADC1DMABuffer[2];

void _dma1_channel1_isr()
{
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        DMA1_Channel1->CCR &= ~DMA_CCR1_EN; // Disable DMA channel

        ubADC1DMAComplete = 1;

        DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1; // Clear all interrupt flags otherwise DMA won't start again
        DMA1_Channel1->CNDTR = 2;
        DMA1_Channel1->CMAR = (uint32_t)pusADC1DMABuffer;
        DMA1_Channel1->CCR |= DMA_CCR1_EN; // Enable DMA channel
    }
}
void _adc3_isr()
{
    if(ADC3->SR & ADC_SR_AWD)
    {
        if(ADC3->HTR == ADC_MAX_OUTPUT_CODE) // If we were searching for low battery
        {
            g_ubBatteryLow = 1;

            ADC3->HTR = (uint16_t)((LOW_BAT_VOLTAGE + LOW_BAT_VOLTAGE_HYST) / BAT_VOLT_DIV / ADC_VOLTAGE_STEP); // High threshold
            ADC3->LTR = 0; // Low threshold
        }
        else
        {
            g_ubBatteryLow = 0;

            ADC3->HTR = ADC_MAX_OUTPUT_CODE; // High threshold
            ADC3->LTR = (uint16_t)(LOW_BAT_VOLTAGE / BAT_VOLT_DIV / ADC_VOLTAGE_STEP); // Low threshold
        }

        ADC3->SR &= ~ADC_SR_AWD; // Clear the flag
    }
}
void adc_init()
{
    // ADC1
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC peripheral clock

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable DMA1 peripheral clock

    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_EXTTRIG | (7 << 17) | ADC_CR2_DMA; // EXTRIG = Software Trigger
    ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC

    delay_ms(1); // Wait for the ADC to stabilize

    ADC1->CR2 |= ADC_CR2_RSTCAL; // Reset calibration
    while(ADC1->CR2 & ADC_CR2_RSTCAL); // Wait for it to reset

    ADC1->CR2 |= ADC_CR2_CAL; // Start calibration
    while(ADC1->CR2 & ADC_CR2_CAL); // Wait for it to complete

    ADC1->SQR1 = (1 << 20); // Run 2 conversions
    ADC1->SQR3 = (WATER_TEMP_SENSOR_2_ADC_CH << 5) | (WATER_TEMP_SENSOR_1_ADC_CH << 0); // First channel 14 and then 15
    ADC1->SMPR1 = (3 << ((WATER_TEMP_SENSOR_2_ADC_CH % 10) * 3)) | (3 << ((WATER_TEMP_SENSOR_1_ADC_CH % 10) * 3)); // 28.5 cycles sampling time for channels 14 & 15

    DMA1_Channel1->CNDTR = 2;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)pusADC1DMABuffer;
    DMA1_Channel1->CCR = (0 << 12) | (1 << 10) | (1 << 8) | DMA_CCR1_MINC | DMA_CCR1_TCIE | DMA_CCR1_EN; // Low priority, 16-bit transfers, Memory increment, Transfer Complete interrupt, Enable

    IRQ_SET_PRIO(DMA1_Channel1_IRQn, 5, 1);
    IRQ_ENABLE(DMA1_Channel1_IRQn);

    // ADC3
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC3RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC3RST;
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;

    ADC3->CR1 = ADC_CR1_AWDEN | ADC_CR1_AWDIE; // Enable watchdog on all (one) regular channels
    ADC3->CR2 = ADC_CR2_CONT; // Continuous conversion
    ADC3->CR2 |= ADC_CR2_ADON; // Enable ADC
    ADC3->HTR = ADC_MAX_OUTPUT_CODE; // High threshold
    ADC3->LTR = (uint16_t)(LOW_BAT_VOLTAGE / BAT_VOLT_DIV / ADC_VOLTAGE_STEP); // Low threshold

    delay_ms(1); // Wait for the ADC to stabilize

    ADC3->CR2 |= ADC_CR2_RSTCAL; // Reset calibration
    while(ADC3->CR2 & ADC_CR2_RSTCAL); // Wait for it to reset

    ADC3->CR2 |= ADC_CR2_CAL; // Start calibration
    while(ADC3->CR2 & ADC_CR2_CAL); // Wait for it to complete

    ADC3->SQR1 = (0 << 20); // Run 1 conversions
    ADC3->SQR3 = (BAT_VOLT_ADC_CH << 0); // Channel 2
    ADC3->SMPR2 = (3 << ((BAT_VOLT_ADC_CH % 10) * 3)); // 28.5 cycles sampling time for channel 2

    ADC3->CR2 |= ADC_CR2_ADON; // Start conversions

    IRQ_SET_PRIO(ADC3_IRQn, 5, 0);
    IRQ_ENABLE(ADC3_IRQn);
}
void adc_wakeup()
{
    ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC1
    ADC3->CR2 |= ADC_CR2_ADON; // Enable ADC3

    delay_ms(1); // Wait for the ADC to stabilize

    ADC3->CR2 |= ADC_CR2_ADON; // Start conversions on ADC3 (Battery voltage)
}
void adc_sleep()
{
    ADC1->CR2 &= ~ADC_CR2_ADON; // Disable ADC1
    ADC3->CR2 &= ~ADC_CR2_ADON; // Disable ADC3
}
void adc_read_water_temp(double *pgData, uint32_t ulSamples)
{
    static uint8_t ubFilterInit = 0;
    static double gWaterTemperature[2] = {0, 0};
    double gCurrentWaterTemperature[2] = {0, 0};

    for(uint32_t i = 0; i < ulSamples; i++)
    {
        ubADC1DMAComplete = 0;
        ADC1->CR2 |= ADC_CR2_SWSTART;

        while(!ubADC1DMAComplete);

        gCurrentWaterTemperature[0] += pusADC1DMABuffer[0];
        gCurrentWaterTemperature[1] += pusADC1DMABuffer[1];
    }

    gCurrentWaterTemperature[0] /= ulSamples;
    gCurrentWaterTemperature[1] /= ulSamples;

    if(!ubFilterInit)
    {
        gWaterTemperature[0] = gCurrentWaterTemperature[0];
        gWaterTemperature[1] = gCurrentWaterTemperature[1];

        ubFilterInit = 1;
    }
    else
    {
        gWaterTemperature[0] = gCurrentWaterTemperature[0] = (gWaterTemperature[0] * (WATER_TEMP_SENSOR_FILTER_SIZE - 1) + gCurrentWaterTemperature[0]) / WATER_TEMP_SENSOR_FILTER_SIZE;
        gWaterTemperature[1] = gCurrentWaterTemperature[1] = (gWaterTemperature[1] * (WATER_TEMP_SENSOR_FILTER_SIZE - 1) + gCurrentWaterTemperature[1]) / WATER_TEMP_SENSOR_FILTER_SIZE;
    }

    gCurrentWaterTemperature[0] *= ADC_VOLTAGE_STEP;
    gCurrentWaterTemperature[1] *= ADC_VOLTAGE_STEP;

    pgData[0] = LMT8X_TRANSFER_FUNC(LMT86_KA, LMT86_KB, LMT86_KC, gCurrentWaterTemperature[0]);
    pgData[1] = LMT8X_TRANSFER_FUNC(LMT86_KA, LMT86_KB, LMT86_KC, gCurrentWaterTemperature[1]);
}
void adc_read_battery(double *pgData, uint32_t ulSamples)
{
    static uint8_t ubFilterInit = 0;
    static double gBatteryVoltage = 0;
    double gCurrentBatteryVoltage = 0;

    ADC3->SR &= ~ADC_SR_EOC; // Force new conversion

    for(uint32_t i = 0; i < ulSamples; i++)
    {
        while(!(ADC3->SR & ADC_SR_EOC));

        gCurrentBatteryVoltage += ADC3->DR;
    }

    gCurrentBatteryVoltage /= ulSamples;

    if(!ubFilterInit)
    {
        gBatteryVoltage = gCurrentBatteryVoltage;

        ubFilterInit = 1;
    }
    else
    {
        gBatteryVoltage = gCurrentBatteryVoltage = (gBatteryVoltage * (BAT_VOLT_FILTER_SIZE - 1) + gCurrentBatteryVoltage) / BAT_VOLT_FILTER_SIZE;
    }

    gCurrentBatteryVoltage *= ADC_VOLTAGE_STEP;

    pgData[0] = gCurrentBatteryVoltage * BAT_VOLT_DIV;
}
