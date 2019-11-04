#include "leds.h"

void leds_init()
{
    // TODO: setup scroll wheel led timer

    RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable TIM1 peripheral clock

    while(TIM1->CR1 & TIM_CR1_CEN)
        TIM1->CR1 &= ~TIM_CR1_CEN; // Disable the timer

    // timer configuration
    TIM1->ARR = (1 << 8) - 1; // 8-bit PWM
    TIM1->PSC = 6; // frequency = [APB1_TIM_CLOCK_FREQ / TIM1->PSC + 1 / TIM1->ARR] = [36MHz / 7 / 255] =  20.168KHz
    TIM1->EGR |= TIM_EGR_UG; // Update immediatly

    // capture compare configuration
    TIM1->CCMR1 = (6 << 4) | (6 << 12); // PWM Mode 1 (CH1 & CH2)
    TIM1->CCMR2 = (6 << 4); // PWM Mode 1 (CH3)
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E; // Enable CH1, CH2, CH3, active high

    // capture compare values
    TIM1->CCR1 = 0; // capture compare ch1, sets green duty cycle (0-255)
    TIM1->CCR2 = 0; // capture compare ch2, sets red duty cycle (0-255)
    TIM1->CCR3 = 0; // capture compare ch3, sets blue duty cycle (0-255)

    TIM1->BDTR = TIM_BDTR_MOE; // Main output enable

    TIM1->CR1 |= TIM_CR1_CEN; // enable timer
}
void leds_logo_set(uint8_t ubRed, uint8_t ubGreen, uint8_t ubBlue)
{
    LOGO_RED_DUTY = CLIP(0, ubRed, 255);
    LOGO_GREEN_DUTY = CLIP(0, ubGreen, 255);
    LOGO_BLUE_DUTY = CLIP(0, ubBlue, 255);
}
void leds_scroll_set(uint8_t ubRed, uint8_t ubGreen, uint8_t ubBlue)
{
    // TODO: setup set func. for scroll wheel 
    LOGO_RED_DUTY = CLIP(0, ubRed, 255);
    LOGO_GREEN_DUTY = CLIP(0, ubGreen, 255);
    LOGO_BLUE_DUTY = CLIP(0, ubBlue, 255);
}