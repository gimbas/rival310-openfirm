#ifndef __I2C_H__
#define __I2C_H__

#include <stm32f10x.h>
#include "utils.h"
#include "rcc.h"

#define I2C_NORMAL 0
#define I2C_FAST 1

#define I2C_RESTART 0
#define I2C_STOP 1

void i2c1_init(uint8_t ubMode);
uint8_t i2c1_transmit(uint8_t ubAddress, uint8_t* pubSrc, uint32_t ulCount, uint8_t ubStop);
static inline uint8_t i2c1_write(uint8_t ubAddress, uint8_t* pubSrc, uint32_t ulCount, uint8_t ubStop)
{
    return i2c1_transmit((ubAddress << 1) & ~0x01, pubSrc, ulCount, ubStop);
}
static inline uint8_t i2c1_write_byte(uint8_t ubAddress, uint8_t ubData, uint8_t ubStop)
{
    return i2c1_transmit((ubAddress << 1) & ~0x01, &ubData, 1, ubStop);
}
static inline uint8_t i2c1_read(uint8_t ubAddress, uint8_t* pubDst, uint32_t ulCount, uint8_t ubStop)
{
    return i2c1_transmit((ubAddress << 1) | 0x01, pubDst, ulCount, ubStop);
}
static inline uint8_t i2c1_read_byte(uint8_t ubAddress, uint8_t ubStop)
{
    uint8_t ubData = 0x00;

    i2c1_transmit((ubAddress << 1) | 0x01, &ubData, 1, ubStop);

    return ubData;
}

void i2c2_init(uint8_t ubMode);
uint8_t i2c2_transmit(uint8_t ubAddress, uint8_t* pubSrc, uint32_t ulCount, uint8_t ubStop);
static inline uint8_t i2c2_write(uint8_t ubAddress, uint8_t* pubSrc, uint32_t ulCount, uint8_t ubStop)
{
    return i2c2_transmit((ubAddress << 1) & ~0x01, pubSrc, ulCount, ubStop);
}
static inline uint8_t i2c2_write_byte(uint8_t ubAddress, uint8_t ubData, uint8_t ubStop)
{
    return i2c2_transmit((ubAddress << 1) & ~0x01, &ubData, 1, ubStop);
}
static inline uint8_t i2c2_read(uint8_t ubAddress, uint8_t* pubDst, uint32_t ulCount, uint8_t ubStop)
{
    return i2c2_transmit((ubAddress << 1) | 0x01, pubDst, ulCount, ubStop);
}
static inline uint8_t i2c2_read_byte(uint8_t ubAddress, uint8_t ubStop)
{
    uint8_t ubData = 0x00;

    i2c2_transmit((ubAddress << 1) | 0x01, &ubData, 1, ubStop);

    return ubData;
}


#endif
