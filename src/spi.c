#include "spi.h"

void spi1_init(uint8_t ubMode, uint8_t ubClockDiv, uint8_t ubBitMode)
{
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; // Reset peripheral
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable SPI peripheral clock

    SPI1->I2SCFGR = 0x0000;

    SPI1->CR2 = 0x00000000;
    SPI1->CR1 = (uint32_t)(ubMode | ubClockDiv | ubBitMode) | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;
}
uint8_t spi1_transfer_byte(const uint8_t ubData)
{
    while(!(SPI1->SR & SPI_SR_TXE));

    SPI1->DR = ubData;

    while(!(SPI1->SR & SPI_SR_RXNE));

    return (uint8_t)SPI1->DR;
}
void spi1_transfer(const uint8_t* pubSrc, uint32_t ulSize, uint8_t* pubDst)
{
    if(pubSrc)
    {
        while(ulSize--)
        {
            if(pubDst)
                *(pubDst++) = spi1_transfer_byte(*(pubSrc++));
            else
                spi1_transfer_byte(*(pubSrc++));
        }
    }
    else if(pubDst)
    {
        while(ulSize--)
            *(pubDst++) = spi1_transfer_byte(0x00);
    }
}

void spi2_init(uint8_t ubMode, uint8_t ubClockDiv, uint8_t ubBitMode)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST; // Reset peripheral
    RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // Enable SPI peripheral clock

    SPI2->I2SCFGR = 0x0000;

    SPI2->CR2 = 0x00000000;
    SPI2->CR1 = (uint32_t)(ubMode | ubClockDiv | ubBitMode) | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;
}
uint8_t spi2_transfer_byte(const uint8_t ubData)
{
    while(!(SPI2->SR & SPI_SR_TXE));

    SPI2->DR = ubData;

    while(!(SPI2->SR & SPI_SR_RXNE));

    return (uint8_t)SPI2->DR;
}
void spi2_transfer(const uint8_t* pubSrc, uint32_t ulSize, uint8_t* pubDst)
{
    if(pubSrc)
    {
        while(ulSize--)
        {
            if(pubDst)
                *(pubDst++) = spi2_transfer_byte(*(pubSrc++));
            else
                spi2_transfer_byte(*(pubSrc++));
        }
    }
    else if(pubDst)
    {
        while(ulSize--)
            *(pubDst++) = spi2_transfer_byte(0x00);
    }
}
