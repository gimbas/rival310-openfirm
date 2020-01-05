#include "adc.h"

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
}
void adc_wakeup()
{
    ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC1

    delay_ms(1); // Wait for the ADC to stabilize
}
void adc_sleep()
{
    ADC1->CR2 &= ~ADC_CR2_ADON; // Disable ADC1
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
}