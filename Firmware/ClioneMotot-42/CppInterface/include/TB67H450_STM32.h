//
// Created by 0209 on 2026/1/10.
//

#ifndef TB67H450_STM32_H
#define TB67H450_STM32_H

#include "tb67h450_base.h"
#include "Cpphand.h"

class TB67H450_STM32 : public TB67H450Base{
public:
    TB67H450_STM32() = default;
protected:
    void InitGpio() override;
    void InitPwm() override;

    void DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits) override;

    void SetInputA(bool _statusAp, bool _statusAm) override;

    void SetInputB(bool _statusBp, bool _statusBm) override;
};



#endif //TB67H450_STM32_H
