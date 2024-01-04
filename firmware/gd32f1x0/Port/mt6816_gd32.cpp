#include "mt6816_gd32.h"
#include "spi.h"

void MT6816::SpiInit()
{
    
}

uint16_t MT6816::SpiTransmitAndRead16Bits(uint16_t _dataTx)
{
    uint16_t dataRx;

    spi_cs_enable();
    dataRx = spi_readwrite_halfworld(_dataTx);
    spi_cs_disable();

    return dataRx;
}
