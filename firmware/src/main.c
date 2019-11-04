#include <stm32f10x.h>
#include <stdlib.h>
#include <math.h>
#include "debug_macros.h"
#include "nvic.h"
#include "utils.h"
#include "atomic.h"
#include "systick.h"
#include "rcc.h"
#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "rtc.h"
#include "crc.h"
#include "flash.h"

// Structs
typedef struct
{
    volatile uint16_t usID0_15;
    volatile uint16_t usID16_31;
    volatile uint32_t ulID32_63;
    volatile uint32_t ulID64_96;
} system_unique_id_t;

// Internal Flash Addresses
#define FLASH_APP_ADDRESS           (FLASH_BASE + 0x8000)

// Forward declarations
static void reset() __attribute__((noreturn));
static void sleep();

static uint32_t get_free_ram();

// Variables
extern system_unique_id_t _system_unique_id;

// ISRs
void _nmi_isr()
{
    // The NMI is used for the Clock Security System (CSS)
    rcc_update_clocks();

    //if(rfm69_init(RADIO_FLOAT_NODE_ID, RADIO_NETWORK_ID, RADIO_AES_KEY))
    //    DBGPRINTLN_CTX("RFM69 init OK!");
    //else
    //    DBGPRINTLN_CTX("RFM69 init NOK!");cks(); // Update clocks, we should be running on HSI now

    systick_init();

//    uart1_init(500000);

//    spi1_init(0, SPI_CLOCK_DIV_2, SPI_MSB_FIRST);
//    i2c2_init(I2C_NORMAL);

    DBGPRINTLN_CTX("HSE Clock failed, switched to HSI!");
    DBGPRINTLN_CTX("RCC - System Clock: %.1f MHz!", (float)SYS_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - AHB Clock: %.1f MHz!", (float)AHB_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB1 Clock: %.1f MHz!", (float)APB1_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB2 Clock: %.1f MHz!", (float)APB2_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB1 Timers Clock: %.1f MHz!", (float)APB1_TIM_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB2 Timers Clock: %.1f MHz!", (float)APB2_TIM_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - ADC Clock: %.1f MHz!", (float)ADC_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("UART1 - Baud: %lu bps!", 500000);
    DBGPRINTLN_CTX("SPI1 - Clock: %.1f MHz!", (float)APB2_CLOCK_FREQ / 1000000 / 2);
    DBGPRINTLN_CTX("I2C2 - Clock: %lu kHz!", 100);

    RCC->CIR |= RCC_CIR_CSSC;

    DBGPRINTLN_CTX("Resetting system in 2s...");

    delay_ms(2000);

    reset();
}
void _exti0_isr()
{
    if(EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR = EXTI_PR_PR0;

//        rfm69_isr();
    }
}
void _exti15_10_isr()
{
    if(EXTI->PR & EXTI_PR_PR12)
    {
        EXTI->PR = EXTI_PR_PR12;

//      Call Accelerometer ISR
    }
}

// Functions
void reset()
{
    SCB->AIRCR = 0x05FA0000 | _VAL2FLD(SCB_AIRCR_SYSRESETREQ, 1);

    while(1);
}
void sleep()
{
    DBGPRINTLN_CTX("Sleeping peripherals...");

//    led_init(0); // Init the LED in IO mode
//    LED_OFF(); // Set LED off

    adc_sleep(); // Sleep the ADCs
//    adxl345_sleep(); // Sleep the accelerometer
//    spi_flash_power_down(1); // Power down the flash

    DBGPRINTLN_CTX("Sleeping radio...");

//    rfm69_listen_mode(); // Put the radio in listen mode

    SCB->SCR |= _VAL2FLD(SCB_SCR_SLEEPDEEP, 1); // CPU deep sleep
    PWR->CR &= ~PWR_CR_PDDS; // Stop mode
    PWR->CR |= PWR_CR_LPDS; // Regulator in Low-power mode

    DBGPRINTLN_CTX("Going to sleep after mem transactions");

    delay_ms(20);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        __DMB(); // Wait for all memory transactions to finish before memory access
        __DSB(); // Wait for all memory transactions to finish before executing instructions
        __ISB(); // Wait for all memory transactions to finish before fetching instructions
        __SEV(); // Set the event flag to ensure the next WFE will be a NOP
        __WFE(); // NOP and clear the event flag
        __WFE(); // Wait for event
        __NOP(); // Prevent debugger crashes

        rcc_init(); // Re-configure clocks as we wake up running on HSI

        DBGPRINTLN_CTX("Just woke up, running ISR...");
    }

    DBGPRINTLN_CTX("Finished wakeup ISR");

    DBGPRINTLN_CTX("Waking up peripherals...");

    adc_wakeup(); // Enable the ADCs
//    adxl345_wakeup(); // Start the accelerometer
//    spi_flash_power_down(0); // Wake up the flash
}

uint32_t get_free_ram()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        extern void *_sbrk(int);

        void *pCurrentHeap = _sbrk(1);

        if(!pCurrentHeap)
            return 0;

        uint32_t ulFreeRAM = (uint32_t)__get_MSP() - (uint32_t)pCurrentHeap;

        _sbrk(-1);

        return ulFreeRAM;
    }
}

int init()
{
    rcc_init(); // Switch to HSE/PLL
    rcc_update_clocks(); // Update clock values

    systick_init(); // Init system tick

    gpio_init(); // Init GPIOs

//    adc_init(); // Init ADCs
    rtc_init(); // Init RTC
    crc_init(); // Init CRC unit

//    led_init(ubLEDMode); // Init the LED in PWM mode

//    uart1_init(500000);

//    spi1_init(0, SPI_CLOCK_DIV_2, SPI_MSB_FIRST);
//    i2c2_init(I2C_NORMAL);

    DBGPRINTLN_CTX("SeaFloat Float Node v%lu (%s %s)!", BUILD_VERSION, __DATE__, __TIME__);
    DBGPRINTLN_CTX("Interfaces init OK!");
    DBGPRINTLN_CTX("Device ID: 0x%03X", DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID);
    DBGPRINTLN_CTX("Device Revision: 0x%04X", (DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID) >> 16);
    DBGPRINTLN_CTX("Flash Size: %hu kB", FLASH_SIZE >> 10);
    DBGPRINTLN_CTX("Free RAM: %lu B", get_free_ram());
    DBGPRINTLN_CTX("Unique ID: %04X-%04X-%08X-%08X", _system_unique_id.usID0_15, _system_unique_id.usID16_31, _system_unique_id.ulID32_63, _system_unique_id.ulID64_96);

    DBGPRINTLN_CTX("RCC - System Clock: %.1f MHz!", (float)SYS_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - AHB Clock: %.1f MHz!", (float)AHB_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB1 Clock: %.1f MHz!", (float)APB1_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB2 Clock: %.1f MHz!", (float)APB2_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB1 Timers Clock: %.1f MHz!", (float)APB1_TIM_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - APB2 Timers Clock: %.1f MHz!", (float)APB2_TIM_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("RCC - ADC Clock: %.1f MHz!", (float)ADC_CLOCK_FREQ / 1000000);
    DBGPRINTLN_CTX("UART1 - Baud: %lu bps!", 500000);
    DBGPRINTLN_CTX("SPI1 - Clock: %.1f MHz!", (float)APB2_CLOCK_FREQ / 1000000 / 2);
    DBGPRINTLN_CTX("I2C2 - Clock: %lu kHz!", 100);

    delay_ms(50);

//    DBGPRINTLN_CTX("Scanning I2C bus 2...");

//    for(uint8_t a = 0x0E; a < 0x80; a++)
//        if(i2c2_write(a, 0, 0, I2C_STOP))
//            DBGPRINTLN_CTX("  Address 0x%02X ACKed!", a);


//    if(spi_flash_init())
//        DBGPRINTLN_CTX("SPI Flash init OK!");
//    else
//        DBGPRINTLN_CTX("SPI Flash init NOK!");


    return 0;
}
int main()
{

    // Config peripherals
//    spi_flash_write_status_3((spi_flash_read_status_3() & 0x9B) | 0x04); // Enable individual block/sector protection (WPS bit set) and full drive strength

//    DBGPRINTLN_CTX("SPI Flash Unique ID: %016llX", spi_flash_read_unique_id());

//    DBGPRINTLN_CTX("Radio carrier: %.3f MHz", rfm69_get_carrier() / 1000000.f);
//    DBGPRINTLN_CTX("Radio TX bandwidth: %.2f kHz", (rfm69_get_deviation() * 2) / 1000.f);
//    DBGPRINTLN_CTX("Radio bit rate: %lu bps", rfm69_get_bit_rate());

    // Radio Callbacks
//    rfm69_set_timeout_callback(radio_timeout_callback);
//    rfm69_set_tx_callback(radio_tx_callback);
//    rfm69_set_ack_callback(radio_ack_callback);
//    rfm69_set_rx_callback(radio_rx_callback);

    // Radio config
//    rfm69_set_atc_target_rssi(RADIO_GATEWAY_ID, 0); // Enable ATC, target power 0 so the power is always at max

    for(;;)
    {
        //rfm69_tick();

        static uint64_t ullLastBlinkTick = 0;

        if(g_ullSystemTick - ullLastBlinkTick >= 1)
        {
            ullLastBlinkTick = g_ullSystemTick;

            static uint8_t state = 0;

            switch(state) // LED state machine
            {
                case 0:
                {
                    if(LED_DUTY < 1023)
                        LED_DUTY++;

                    if(LED_DUTY >= 1023)
                        state = 1;
                }
                break;
                case 1:
                {
                    if(LED_DUTY)
                        LED_DUTY--;

                    if(!LED_DUTY)
                        state = 2;
                }
                break;
                case 2:
                {
                    static uint32_t delay = 0;

                    if(delay++ >= 300)
                    {
                        state = 0;
                        delay = 0;
                    }
                }
                break;
            }
        }

/*        if(g_ullSystemTick >= ullSleepTick && rfm69_tx_empty())
        {
            ullSleepTick = g_ullSystemTick + ulSleepTimeout;

            spi_flash_protect_block(SPI_FLASH_UPDATE_APP_BASE + 0 * 16 * SPI_FLASH_SECTOR_SIZE); // Protect node update app regions
            spi_flash_protect_block(SPI_FLASH_UPDATE_APP_BASE + 1 * 16 * SPI_FLASH_SECTOR_SIZE);
            spi_flash_protect_block(SPI_FLASH_UPDATE_APP_BASE + 2 * 16 * SPI_FLASH_SECTOR_SIZE);
            spi_flash_protect_block(SPI_FLASH_UPDATE_APP_BASE + 3 * 16 * SPI_FLASH_SECTOR_SIZE);

            ulFOTAUpdateAppVersion = 0;
            ulFOTAAppSize = 0;
            ulFOTAAppSizeReceived = 0;
            ulFOTAAppTimestamp = 0;
            usFOTAPacketID = 0;

            if(ulDataReportDelay)
            {
                ubDataUpdated = 0;

                rtc_set_alarm(rtc_get_time() + ulDataReportDelay);
            }
            else
            {
                rtc_set_alarm(rtc_get_time() + 60);
            }

            sleep();

            if(rtc_get_alarm() <= rtc_get_time() && !ulDataReportDelay) // Assume radio locked up, and try to unlock it
            {
                DBGPRINTLN_CTX("Unexpected wake up by RTC, resetting radio...");

                rfm69_set_mode(RFM69_RF_OPMODE_STANDBY); // Standby

                delay_ms(10);

                rfm69_set_mode(RFM69_RF_OPMODE_RECEIVER); // RX
            }
        }
*/
    }

    return 0;
}
