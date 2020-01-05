#include "crc.h"

#ifdef CRC_IMPL_SOFTWARE

void crc_init()
{

}
uint32_t calc_crc32(uint8_t *pData, uint32_t ulSize)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t rem = ulSize % 4;
    uint32_t data;

    ulSize -= rem;

    while(ulSize)
    {
        data = *(uint32_t *)pData;

        crc ^= data;

        for(uint8_t i = 0; i < 32; i++)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc = (crc << 1);
        }

        pData += 4;
        ulSize -= 4;
    }

    if(!rem)
        return crc;

    data = 0;

    for(uint8_t i = 0; i < rem; i++)
        data |= *pData++ << (8 * i);

    crc ^= data;

    for(uint8_t i = 0; i < 32; i++)
    {
        if (crc & 0x80000000)
            crc = (crc << 1) ^ 0x04C11DB7;
        else
            crc = (crc << 1);
    }

    return crc;
}

#else

void crc_init()
{
    RCC->AHBENR |= RCC_AHBENR_CRCEN;
}
uint32_t calc_crc32(uint8_t *pData, uint32_t ulSize)
{
    uint8_t rem = ulSize % 4;

    ulSize -= rem;

    CRC->CR = CRC_CR_RESET;

    while(ulSize)
    {
        CRC->DR = *(uint32_t *)pData;

        pData += 4;
        ulSize -= 4;
    }

    if(!rem)
        return CRC->DR;

    uint32_t data = 0;

    for(uint8_t i = 0; i < rem; i++)
        data |= *pData++ << (8 * i);

    CRC->DR = data;

    return CRC->DR;
}

#endif
