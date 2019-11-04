#ifndef __SPI_H__
#define __SPI_H__

#include <stm32f10x.h>
#include <stdlib.h>

#define SPI_CLOCK_DIV_2     0x00
#define SPI_CLOCK_DIV_4     0x08
#define SPI_CLOCK_DIV_8     0x10
#define SPI_CLOCK_DIV_16    0x18
#define SPI_CLOCK_DIV_32    0x20
#define SPI_CLOCK_DIV_64    0x28
#define SPI_CLOCK_DIV_128   0x30
#define SPI_CLOCK_DIV_256   0x38

#define SPI_MSB_FIRST 0x00
#define SPI_LSB_FIRST 0x80

void spi1_init(uint8_t ubMode, uint8_t ubClockDiv, uint8_t ubBitMode);
uint8_t spi1_transfer_byte(const uint8_t ubData);
void spi1_transfer(const uint8_t* pubSrc, uint32_t ulSize, uint8_t* pubDst);
static inline void spi1_write(const uint8_t* pubSrc, uint32_t ulSize)
{
    spi1_transfer(pubSrc, ulSize, NULL);
}
static inline void spi1_read(uint8_t* pubDst, uint32_t ulSize)
{
    spi1_transfer(NULL, ulSize, pubDst);
}

void spi2_init(uint8_t ubMode, uint8_t ubClockDiv, uint8_t ubBitMode);
uint8_t spi2_transfer_byte(const uint8_t ubData);
void spi2_transfer(const uint8_t* pubSrc, uint32_t ulSize, uint8_t* pubDst);
static inline void spi2_write(const uint8_t* pubSrc, uint32_t ulSize)
{
    spi2_transfer(pubSrc, ulSize, NULL);
}
static inline void spi2_read(uint8_t* pubDst, uint32_t ulSize)
{
    spi2_transfer(NULL, ulSize, pubDst);
}

#endif
