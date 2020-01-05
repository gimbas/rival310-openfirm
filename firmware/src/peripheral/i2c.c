#include "i2c.h"

void i2c1_init(uint8_t ubMode)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C peripheral clock

    I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C to change settings

    // Analog filters may be stuck at reset
    // This causes BUSY bit to always be set and the bus is locked
    // See Sillicon bugs at ERRATA
    if(I2C1->SR2 & I2C_SR2_BUSY)
    {
        // This sequence unlocks the filters by doing:
        //  SCL & SDA > GPIO OD > HIGH
        //  SDA > LOW
        //  SCL > LOW (START condition)
        //  SCL > HIGH
        //  SDA > HIGH (STOP condition)

        GPIOB->CRL &= 0x00FFFFFF; // Reset PB6 & PB7
        GPIOB->CRL |= 0x66000000; // Set PB6 & PB7 as GPIO output open-drain
        GPIOB->BSRR = (1 << 6) | (1 << 7); // Set PB6 & PB7 high
        while((GPIOB->IDR & ((1 << 6) | (1 << 7))) != ((1 << 6) | (1 << 7))); // Wait for them to go high
        GPIOB->BSRR = (1 << 7) << 16; // Set PB7 (SDA) low
        while(GPIOB->IDR & (1 << 7)); // Wait for it to go low
        GPIOB->BSRR = (1 << 6) << 16; // Set PB6 (SCL) low
        while(GPIOB->IDR & (1 << 6)); // Wait for it to go low
        GPIOB->BSRR = (1 << 6); // Set PB6 (SCL) high
        while(!(GPIOB->IDR & (1 << 6))); // Wait for it to go high
        GPIOB->BSRR = (1 << 7); // Set PB7 (SDA) high
        while(!(GPIOB->IDR & (1 << 7))); // Wait for it to go high

        GPIOB->CRL &= 0x00FFFFFF; // Reset PB6 & PB7
        GPIOB->CRL |= 0xEE000000; // Set PB6 & PB7 as AF output open-drain

        GPIOB->BSRR = ((1 << 6) | (1 << 7)) << 16; // Set the GPIO values to low

        I2C1->CR1 |= I2C_CR1_SWRST;
        I2C1->CR1 &= ~I2C_CR1_SWRST;
    }

    if(ubMode == I2C_NORMAL)
    {
        I2C1->CCR = (uint16_t)(APB1_CLOCK_FREQ / (100000 * 2));
        I2C1->TRISE = (uint16_t)(APB1_CLOCK_FREQ / 1000000) + 1;
    }
    else
    {
        I2C1->CCR = (uint16_t)(APB1_CLOCK_FREQ / (400000 * 3)) | I2C_CCR_FS;
        I2C1->TRISE = (uint16_t)((((APB1_CLOCK_FREQ / 1000000) * (uint16_t)300) / (uint16_t)1000) + (uint16_t)1);
    }

    I2C1->CR2 = (uint16_t)(APB1_CLOCK_FREQ / 1000000);
    I2C1->CR1 = I2C_CR1_PE;
}
uint8_t i2c1_transmit(uint8_t ubAddress, uint8_t* pubSrc, uint32_t ulCount, uint8_t ubStop)
{
    while(I2C1->CR1 & I2C_CR1_STOP);

    I2C1->CR1 |= I2C_CR1_START;

    while(!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = ubAddress;

    while(!(I2C1->SR1 & I2C_SR1_ADDR) && !(I2C1->SR1 & I2C_SR1_AF));

    REG_DISCARD(&I2C1->SR2);

    if(I2C1->SR1 & I2C_SR1_AF)
    {
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;

        return 0;
    }

    if(ulCount)
        do
        {
            if (!(ubAddress & 1)) // Write
            {
                I2C1->DR = *(pubSrc++);

                while(!(I2C1->SR1 & I2C_SR1_TXE) && !(I2C1->SR1 & I2C_SR1_AF));

                if(I2C1->SR1 & I2C_SR1_AF)
                {
                    I2C1->SR1 &= ~I2C_SR1_AF;
                    I2C1->CR1 |= I2C_CR1_STOP;

                    return 0;
                }
            }
            else // Read
            {
                if (ulCount > 1)
                    I2C1->CR1 |= I2C_CR1_ACK;
                else
                    I2C1->CR1 &= ~I2C_CR1_ACK;

                while(!(I2C1->SR1 & I2C_SR1_RXNE));

                *(pubSrc++) = I2C1->DR;
            }
        } while(--ulCount);

    if(ubStop)
        I2C1->CR1 |= I2C_CR1_STOP;

    return 1;
}
