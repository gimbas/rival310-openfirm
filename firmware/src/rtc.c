#include "rtc.h"

void rtc_init()
{
    RCC->APB1RSTR |= RCC_APB1RSTR_PWRRST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_PWRRST;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR peripheral clock

    RCC->APB1RSTR |= RCC_APB1RSTR_BKPRST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_BKPRST;
    RCC->APB1ENR |= RCC_APB1ENR_BKPEN; // Enable BKP peripheral clock

    PWR->CR |= PWR_CR_DBP; // Enable access to the backup registers

    if(!(RCC->BDCR & RCC_BDCR_RTCEN)) // Already enabled? (can happen if system is reset by software request)
    {
        RCC->BDCR = RCC_BDCR_RTCSEL_LSI | RCC_BDCR_RTCEN; // LSI selected as RTC clock, RTC Clock enabled

        RTC->CRL &= ~RTC_CRL_RSF;
        while(!(RTC->CRL & RTC_CRL_RSF)); // Wait for RTC sync

        while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write

        RTC->CRL |= RTC_CRL_CNF; // Enter configuration mode

        uint32_t ulPrescaler = 40000 - 1;

        RTC->PRLH = (ulPrescaler >> 16) & 0xF; // Prescaler
        RTC->PRLL = ulPrescaler & 0xFFFF;

        RTC->CNTH = 0; // Reset count
        RTC->CNTL = 0;

        RTC->ALRH = 0xFFFF; // Reset alarm
        RTC->ALRL = 0xFFFF;

        RTC->CRL &= ~RTC_CRL_CNF; // Write configuration

        while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write
    }

    // NOT NEEDED BECAUSE RTC ALARM IS DIRECTLY CONNECTED TO EXTI 17
    //RTC->CRH = RTC_CRH_ALRIE; // Enable alarm iterrupt

    PWR->CR &= ~PWR_CR_DBP; // Disable access to the backup registers

    EXTI->IMR &= ~EXTI_IMR_MR17; // Disable interupt (EXTI 17)
    EXTI->EMR |= EXTI_EMR_MR17; // Enable event (EXTI 17)
    EXTI->RTSR |= EXTI_RTSR_TR17; // Enable rising trigger (EXTI 17)
    EXTI->FTSR &= ~EXTI_FTSR_TR17; // Disable falling trigger (EXTI 17)
}
uint32_t rtc_get_time()
{
    PWR->CR |= PWR_CR_DBP; // Enable access to the backup registers

    RTC->CRL &= ~RTC_CRL_RSF;
    while(!(RTC->CRL & RTC_CRL_RSF)); // Wait for RTC sync

    while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write

    uint32_t ulTime = (RTC->CNTH << 16) | RTC->CNTL;

    PWR->CR &= ~PWR_CR_DBP; // Disable access to the backup registers

    return ulTime;
}
void rtc_set_time(uint32_t ulTime)
{
    PWR->CR |= PWR_CR_DBP; // Enable access to the backup registers

    RTC->CRL &= ~RTC_CRL_RSF;
    while(!(RTC->CRL & RTC_CRL_RSF)); // Wait for RTC sync

    while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write

    RTC->CRL |= RTC_CRL_CNF; // Enter configuration mode

    RTC->CNTH = (uint16_t)(ulTime >> 16); // Set count
    RTC->CNTL = (uint16_t)(ulTime & 0xFFFF);

    RTC->CRL &= ~RTC_CRL_CNF; // Write configuration

    while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write

    PWR->CR &= ~PWR_CR_DBP; // Disable access to the backup registers
}
uint32_t rtc_get_alarm()
{
    PWR->CR |= PWR_CR_DBP; // Enable access to the backup registers

    RTC->CRL &= ~RTC_CRL_RSF;
    while(!(RTC->CRL & RTC_CRL_RSF)); // Wait for RTC sync

    while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write

    uint32_t ulAlarm = (RTC->ALRH << 16) | RTC->ALRL;

    PWR->CR &= ~PWR_CR_DBP; // Disable access to the backup registers

    return ulAlarm;
}
void rtc_set_alarm(uint32_t ulAlarm)
{
    PWR->CR |= PWR_CR_DBP; // Enable access to the backup registers

    RTC->CRL &= ~RTC_CRL_RSF;
    while(!(RTC->CRL & RTC_CRL_RSF)); // Wait for RTC sync

    while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write

    RTC->CRL |= RTC_CRL_CNF; // Enter configuration mode

    RTC->ALRH = (uint16_t)(ulAlarm >> 16); // Set alarm
    RTC->ALRL = (uint16_t)(ulAlarm & 0xFFFF);

    RTC->CRL &= ~RTC_CRL_CNF; // Write configuration

    while(!(RTC->CRL & RTC_CRL_RTOFF)); // Wait for RTC write

    PWR->CR &= ~PWR_CR_DBP; // Disable access to the backup registers
}
