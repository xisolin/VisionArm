//
// Created by 0209 on 2026/1/10.
//

#ifndef MT6816_SMT32_H
#define MT6816_SMT32_H

#pragma once

#include "MT6816_base.h"
#include "Cpphand.h"

class MT6816_STM32 : public MT6816Base{
public:
    MT6816_STM32(uint16_t* caliTable) : MT6816Base(caliTable) {}

    void SpiInit() override;
    uint16_t SpiTransmitAndRead16Bits(uint16_t txData) override;
};



#endif //MT6816_SMT32_H
