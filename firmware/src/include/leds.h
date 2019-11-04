#ifndef __LEDS_H__
#define __LEDS_H__

#include <stm32f10x.h>
#include "systick.h"
#include "nvic.h"
#include "utils.h"
#include "rcc.h"

// PA8 - Rival 310 - Logo RGB Green
// PA9 - Rival 310 - Logo RGB Red
// PA10 - Rival 310 - Logo RGB Blue

#define LOGO_RED_ON() GPIOA->BSRR = BIT(9)
#define LOGO_RED_OFF() GPIOA->BSRR = BIT(9) << 16
#define LOGO_RED_TOGGLE() GPIOA->ODR ^= BIT(9)
#define LOGO_RED_STATUS() !!(GPIOA->IDR & BIT(9))
#define LOGO_RED_DUTY TIM1->CCR2

#define LOGO_GREEN_ON() GPIOA->BSRR = BIT(8)
#define LOGO_GREEN_OFF() GPIOA->BSRR = BIT(8) << 16
#define LOGO_GREEN_TOGGLE() GPIOA->ODR ^= BIT(8)
#define LOGO_GREEN_STATUS() !!(GPIOA->IDR & BIT(8))
#define LOGO_GREEN_DUTY TIM1->CCR1

#define LOGO_BLUE_ON() GPIOA->BSRR = BIT(10)
#define LOGO_BLUE_OFF() GPIOA->BSRR = BIT(10) << 16
#define LOGO_BLUE_TOGGLE() GPIOA->ODR ^= BIT(10)
#define LOGO_BLUE_STATUS() !!(GPIOA->IDR & BIT(10))
#define LOGO_BLUE_DUTY TIM1->CCR3

void leds_init();
void leds_logo_set(uint8_t ubRed, uint8_t ubGreen, uint8_t ubBlue);
void leds_scroll_set(uint8_t ubRed, uint8_t ubGreen, uint8_t ubBlue);

#endif // __LEDS_H__