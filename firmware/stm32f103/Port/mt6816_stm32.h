#ifndef CTRL_STEP_FW_MT6816_STM32_H
#define CTRL_STEP_FW_MT6816_STM32_H

#include "Sensor/Encoder/mt6816_base.h"
#include "Platform/Memory/stockpile.h"

class MT6816 : public MT6816Base
{
public:
    /*
     * _quickCaliDataPtr is the start address where calibration data stored,
     */
    explicit MT6816() : MT6816Base((uint16_t*) (stockpile_quick_cali.begin_add))
    {}

private:
    void SpiInit() override;

    uint16_t SpiTransmitAndRead16Bits(uint16_t _data) override;
};

#endif
