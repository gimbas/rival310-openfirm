#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include <stm32f10x.h>
#include "atomic.h"
#include "rcc.h"

extern volatile uint64_t g_ullSystemTick;

void systick_init();
void delay_ms(uint64_t ullTicks);

#endif
