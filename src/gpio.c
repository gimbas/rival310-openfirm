#include "gpio.h"

void gpio_init()
{
    // PA0 - (RADIO_IRQ) Input pull-down
    // PA1 - (FLASH_CS) Output 50MHz push-pull
    // PA2 - (BAT_VOLT) Input analog (ADC123_IN2)
    // PA3 -
    // PA4 - (RADIO_CS) Output 10MHz push-pull
    // PA5 - AF (SPI1_SCK) Output 10MHz push-pull
    // PA6 - AF (SPI1_MISO) Input floating
    // PA7 - AF (SPI1_MOSI) Output 10MHz push-pull
    // PA8 -
    // PA9 - AF (USART1_TX) Output 10MHz push-pull
    // PA10 - AF (USART1_RX) Input pull-up
    // PA11 -
    // PA12 -
    // PA13 - AF (SWDIO) Output 50MHz push-pull
    // PA14 - AF (SWCLK) Output 50MHz push-pull
    // PA15 -
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPARST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_IOPARST;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable GPIOA peripheral clock
    GPIOA->CRL = IO_MODE_CFG(0, IO_MODE_IN, IO_CFG_IN_PULL) |
                 IO_MODE_CFG(1, IO_MODE_OUT_10M, IO_CFG_OUT_GP_PP) |
                 IO_MODE_CFG(2, IO_MODE_IN, IO_CFG_IN_ANALOG) |
                 IO_MODE_CFG(4, IO_MODE_OUT_10M, IO_CFG_OUT_GP_PP) |
                 IO_MODE_CFG(5, IO_MODE_OUT_10M, IO_CFG_OUT_AF_PP) |
                 IO_MODE_CFG(6, IO_MODE_IN, IO_CFG_IN_FLOAT) |
                 IO_MODE_CFG(7, IO_MODE_OUT_10M, IO_CFG_OUT_AF_PP);
    GPIOA->CRH = IO_MODE_CFG(9, IO_MODE_OUT_10M, IO_CFG_OUT_AF_PP) |
                 IO_MODE_CFG(10, IO_MODE_IN, IO_CFG_IN_PULL) |
                 IO_MODE_CFG(13, IO_MODE_OUT_50M, IO_CFG_OUT_AF_PP) |
                 IO_MODE_CFG(14, IO_MODE_OUT_50M, IO_CFG_OUT_AF_PP);
    GPIOA->ODR = BIT(1) | BIT(4) | BIT(10);

    // PB0 - (RADIO_RST) Output 2MHz push-pull
    // PB1 -
    // PB2 -
    // PB3 - AF (SWO) Output 50MHz push-pull
    // PB4 - (LED) Output 2MHz push-pull
    // PB5 -
    // PB6 -
    // PB7 -
    // PB8 - (CHRG) Input pull-up
    // PB9 - (CH_DETECT) Input pull-up
    // PB10 - AF (I2C2_SCL) Output 2MHz open-drain
    // PB11 - AF (I2C2_SDA) Output 2MHz open-drain
    // PB12 - (ACCELL_IRQ) Input pull-down
    // PB13 -
    // PB14 -
    // PB15 -
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPBRST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_IOPBRST;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Enable GPIOB peripheral clock
    GPIOB->CRL = IO_MODE_CFG(0, IO_MODE_OUT_2M, IO_CFG_OUT_GP_PP) |
                 IO_MODE_CFG(3, IO_MODE_OUT_50M, IO_CFG_OUT_AF_PP) |
                 IO_MODE_CFG(4, IO_MODE_OUT_2M, IO_CFG_OUT_GP_PP);
    GPIOB->CRH = IO_MODE_CFG(8, IO_MODE_IN, IO_CFG_IN_PULL) |
                 IO_MODE_CFG(9, IO_MODE_IN, IO_CFG_IN_PULL) |
                 IO_MODE_CFG(10, IO_MODE_OUT_2M, IO_CFG_OUT_AF_OD) |
                 IO_MODE_CFG(11, IO_MODE_OUT_2M, IO_CFG_OUT_AF_OD) |
                 IO_MODE_CFG(12, IO_MODE_IN, IO_CFG_IN_PULL);
    GPIOB->ODR = BIT(8) | BIT(9);

    // PC0 -
    // PC1 -
    // PC2 -
    // PC3 -
    // PC4 - (WAATER_TEMP_1) Input analog (ADC12_IN14)
    // PC5 - (WAATER_TEMP_2) Input analog (ADC12_IN15)
    // PC6 -
    // PC7 - AF (BUZZ) Output 2MHz push-pull
    // PC8 -
    // PC9 -
    // PC10 -
    // PC11 -
    // PC12 -
    // PC13 -
    // PC14 -
    // PC15 -
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPCRST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_IOPCRST;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable GPIOC peripheral clock
    GPIOC->CRL = IO_MODE_CFG(4, IO_MODE_IN, IO_CFG_IN_ANALOG) |
                 IO_MODE_CFG(5, IO_MODE_IN, IO_CFG_IN_ANALOG) |
                 IO_MODE_CFG(7, IO_MODE_OUT_2M, IO_CFG_OUT_AF_PP);
    GPIOC->CRH = 0;
    GPIOC->ODR = 0;

    // PD0 -
    // PD1 -
    // PD2 -
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPDRST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_IOPDRST;
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN; // Enable GPIOD peripheral clock
    GPIOD->CRL = 0;
    GPIOD->CRH = 0;
    GPIOD->ODR = 0;

    // External Interrupts
    RCC->APB2RSTR |= RCC_APB2RSTR_AFIORST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_AFIORST;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AFIO peripheral clock
    AFIO->EXTICR[0] = IO_EXTI_CFG(0, IO_EXTI_PORTA);
    AFIO->EXTICR[1] = 0;
    AFIO->EXTICR[2] = 0;
    AFIO->EXTICR[3] = IO_EXTI_CFG(12, IO_EXTI_PORTB);

    // EXTI 0 - RFM69 IRQ Line
    EXTI->IMR |= EXTI_IMR_MR0; // Enable interrupt
    EXTI->EMR |= EXTI_EMR_MR0; // Enable event
    EXTI->RTSR |= EXTI_RTSR_TR0; // Enable rising trigger
    EXTI->FTSR &= ~EXTI_FTSR_TR0; // Disable falling trigger
    IRQ_SET_PRIO(EXTI0_IRQn, 1, 0);
    IRQ_CLEAR(EXTI0_IRQn);
    IRQ_ENABLE(EXTI0_IRQn);

    // EXTI 10 - N/A
    // EXTI 11 - N/A
    // EXTI 12 - Accelerometer IRQ Line
    // EXTI 13 - N/A
    // EXTI 14 - N/A
    // EXTI 15 - N/A
    EXTI->IMR |= EXTI_IMR_MR12; // Enable interrupt
    EXTI->EMR |= EXTI_EMR_MR12; // Enable event
    EXTI->RTSR |= EXTI_RTSR_TR12; // Enable rising trigger
    EXTI->FTSR &= ~EXTI_FTSR_TR12; // Disable falling trigger
    IRQ_SET_PRIO(EXTI15_10_IRQn, 1, 0);
    IRQ_CLEAR(EXTI15_10_IRQn);
    IRQ_ENABLE(EXTI15_10_IRQn);

    // Remapping
    AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE | AFIO_MAPR_TIM3_REMAP_PARTIALREMAP; // Disable JTAG and remap TIM3
}

void led_init(uint8_t ubMode)
{
    if(!ubMode)
    {
        while(TIM3->CR1 & TIM_CR1_CEN)
            TIM3->CR1 &= ~TIM_CR1_CEN; // Disable the timer

        RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN; // Disable TIM3 peripheral clock

        GPIOB->CRL &= 0xFFF0FFFF; // Reset IO mode
        GPIOB->CRL |= IO_MODE_CFG(4, IO_MODE_OUT_2M, IO_CFG_OUT_GP_PP); // Set IO to GP mode

        return;
    }

    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM3 peripheral clock

    TIM3->ARR = (1 << 10) - 1; // 10-bit PWM
    TIM3->PSC = 0; // Maximum frequency
    TIM3->EGR |= TIM_EGR_UG; // Update immediatly

    TIM3->CCMR1 = (6 << 4); // PWM Mode 1
    TIM3->CCER = TIM_CCER_CC1E; // Enable CH1, active high

    TIM3->CR1 |= TIM_CR1_CEN;

    GPIOB->CRL &= 0xFFF0FFFF; // Reset IO mode
    GPIOB->CRL |= IO_MODE_CFG(4, IO_MODE_OUT_2M, IO_CFG_OUT_AF_PP); // Set IO to AF mode
}

void play_sound(uint16_t usFrequency, uint32_t ulTime)
{
    static uint32_t ubInit = 0;

    if(!ubInit)
    {
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST; // Reset peripheral
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Enable TIM8 peripheral clock

        TIM8->PSC = (APB2_TIM_CLOCK_FREQ / 1000000) - 1; // 1 MHz clock

        TIM8->CCR2 = 0;
        TIM8->CCMR1 = (3 << 12); // Toggle mode
        TIM8->CCER = TIM_CCER_CC2E; // Enable CH3N, active low
        TIM8->BDTR = TIM_BDTR_MOE; // Main output enable
        TIM8->EGR |= TIM_EGR_UG; // Update immediatly

        TIM8->CR1 |= TIM_CR1_CEN; // Enable timer

        ubInit = 1;
    }

    if(!usFrequency)
    {
        GPIOC->CRL &= 0x0FFFFFFF; // Reset IO mode
        GPIOC->CRL |= IO_MODE_CFG(7, IO_MODE_OUT_2M, IO_CFG_OUT_GP_PP); // Set IO to GP mode, detached from the timer

        return;
    }

    TIM8->ARR = (1000000 / (usFrequency << 1)) - 1; // Double the frequency
    TIM8->EGR |= TIM_EGR_UG; // Update immediatly

    GPIOC->CRL &= 0x0FFFFFFF; // Reset IO mode
    GPIOC->CRL |= IO_MODE_CFG(7, IO_MODE_OUT_2M, IO_CFG_OUT_AF_PP); // Set IO to AF mode, attached to the timer

    if(!ulTime)
        return;

    delay_ms(ulTime);

    GPIOC->CRL &= 0x0FFFFFFF; // Reset IO mode
    GPIOC->CRL |= IO_MODE_CFG(7, IO_MODE_OUT_2M, IO_CFG_OUT_GP_PP); // Set IO to GP mode, detached from the timer
}