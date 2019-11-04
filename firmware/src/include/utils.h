#ifndef __UTILS_H__
#define __UTILS_H__

#include <stm32f10x.h>

// Memory sections & aliases
#define IRAM_TEXT __attribute__ ((section(".iram0.text")))

// Macro to make a dummy read
#define REG_DISCARD(reg) __asm__ volatile ("" : : "r" (*(volatile uint32_t *)(reg)))

// Macro to check if the address has a valid app
#define IS_VALID_APP(addr) ((*(volatile uint32_t *)(addr) & 0xFFFE0000) == SRAM_BASE)

// Macro to get the bit value
#define BIT(x) (1 << (x))

// Printf macros to print bits
#define UINT8BITSTR         "%c%c%c%c%c%c%c%c"
#define UINT16BITSTR        UINT8BITSTR UINT8BITSTR
#define UINT32BITSTR        UINT16BITSTR UINT16BITSTR
#define UINT64BITSTR        UINT32BITSTR UINT32BITSTR
#define UINT82BITSTR(b)     ((b) & 0x80 ? '1' : '0'), ((b) & 0x40 ? '1' : '0'), ((b) & 0x20 ? '1' : '0'), ((b) & 0x10 ? '1' : '0'), ((b) & 0x08 ? '1' : '0'), ((b) & 0x04 ? '1' : '0'), ((b) & 0x02 ? '1' : '0'), ((b) & 0x01 ? '1' : '0')
#define UINT162BITSTR(b)    UINT82BITSTR(((b) >> 8) & 0xFF), UINT82BITSTR(((b) >> 0) & 0xFF)
#define UINT322BITSTR(b)    UINT162BITSTR(((b) >> 16) & 0xFFFF), UINT162BITSTR(((b) >> 0) & 0xFFFF)
#define UINT642BITSTR(b)    UINT322BITSTR(((b) >> 32) & 0xFFFFFFFF), UINT322BITSTR(((b) >> 0) & 0xFFFFFFFF)

// Sign of, 1 if (+) 0 if (-)
#define SIGN(a)  (!(a < 0))

// Absolute value of
#define ABS(a)  ((a) < 0 ? (-(a)) : (a))

// Macros that get the lesser/greater of two values
#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define CLIP(l,a,h) (MAX(MIN(h,a),l))

// Swap two variables
#define SWAP(a, b)  do{ typeof(a) SWAP = a; a = b; b = SWAP; }while(0)

#endif
