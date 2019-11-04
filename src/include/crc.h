#ifndef __CRC_H__
#define __CRC_H__

#include <stm32f10x.h>

//#define CRC_IMPL_SOFTWARE

void crc_init();
uint32_t calc_crc32(uint8_t *pData, uint32_t ulSize);

#endif
