#ifndef __RCC_H__
#define __RCC_H__

#include <stm32f10x.h>
#include "flash.h"

extern uint32_t SYS_CLOCK_FREQ;
extern uint32_t AHB_CLOCK_FREQ;
extern uint32_t APB1_CLOCK_FREQ;
extern uint32_t APB2_CLOCK_FREQ;
extern uint32_t APB1_TIM_CLOCK_FREQ;
extern uint32_t APB2_TIM_CLOCK_FREQ;
extern uint32_t ADC_CLOCK_FREQ;

void rcc_init();
void rcc_update_clocks();

#endif
