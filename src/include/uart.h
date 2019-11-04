#ifndef __UART_H__
#define __UART_H__

#include <stm32f10x.h>
#include <stdlib.h>
#include <string.h>
#include "nvic.h"
#include "utils.h"
#include "atomic.h"
#include "rcc.h"

#define UART1_DMA_RX_BUFFER_SIZE 128
#define UART1_FIFO_SIZE 256

void uart1_init(uint32_t ulBaud);
void uart1_write_byte(const uint8_t ubData);
uint8_t uart1_read_byte();
uint32_t uart1_available();
void uart1_flush();
static inline void uart1_write(const uint8_t *pubSrc, uint32_t ulSize)
{
    while(ulSize--)
        uart1_write_byte(*pubSrc++);
}
static inline void uart1_read(uint8_t *pubDst, uint32_t ulSize)
{
    while(ulSize--)
        *pubDst++ = uart1_read_byte();
}

#define UART2_DMA_RX_BUFFER_SIZE 128
#define UART2_FIFO_SIZE 256

void uart2_init(uint32_t ulBaud);
void uart2_write_byte(const uint8_t ubData);
uint8_t uart2_read_byte();
uint32_t uart2_available();
void uart2_flush();
static inline void uart2_write(const uint8_t *pubSrc, uint32_t ulSize)
{
    while(ulSize--)
        uart2_write_byte(*pubSrc++);
}
static inline void uart2_read(uint8_t *pubDst, uint32_t ulSize)
{
    while(ulSize--)
        *pubDst++ = uart2_read_byte();
}

#define UART3_DMA_RX_BUFFER_SIZE 128
#define UART3_FIFO_SIZE 256

void uart3_init(uint32_t ulBaud);
void uart3_write_byte(const uint8_t ubData);
uint8_t uart3_read_byte();
uint32_t uart3_available();
void uart3_flush();
static inline void uart3_write(const uint8_t *pubSrc, uint32_t ulSize)
{
    while(ulSize--)
        uart3_write_byte(*pubSrc++);
}
static inline void uart3_read(uint8_t *pubDst, uint32_t ulSize)
{
    while(ulSize--)
        *pubDst++ = uart3_read_byte();
}

#define UART4_DMA_RX_BUFFER_SIZE 128
#define UART4_FIFO_SIZE 256

void uart4_init(uint32_t ulBaud);
void uart4_write_byte(const uint8_t ubData);
uint8_t uart4_read_byte();
uint32_t uart4_available();
void uart4_flush();
static inline void uart4_write(const uint8_t *pubSrc, uint32_t ulSize)
{
    while(ulSize--)
        uart4_write_byte(*pubSrc++);
}
static inline void uart4_read(uint8_t *pubDst, uint32_t ulSize)
{
    while(ulSize--)
        *pubDst++ = uart4_read_byte();
}

void uart5_init(uint32_t ulBaud);
void uart5_write_byte(const uint8_t ubData);
static inline void uart5_write(const uint8_t *pubSrc, uint32_t ulSize)
{
    while(ulSize--)
        uart5_write_byte(*pubSrc++);
}

#endif
