#ifndef __RTC_H__
#define __RTC_H__

#include <stm32f10x.h>
#include "nvic.h"
#include "utils.h"

void rtc_init();
uint32_t rtc_get_time();
void rtc_set_time(uint32_t ulTime);
uint32_t rtc_get_alarm();
void rtc_set_alarm(uint32_t ulAlarm);

#endif
