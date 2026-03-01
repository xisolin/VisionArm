#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#pragma once

#include <array>
#include "Cpphand.h"


class ADC_DRIVER
{
private:
    static constexpr size_t CHANNEL_COUNT = 2;
    static constexpr float ADC_MAX_VALUE = 4095.0f;
    ADC_HandleTypeDef* adc_handle_;
    alignas(4) std::array<uint16_t, CHANNEL_COUNT> adc_buffer_{0};

public:
    explicit ADC_DRIVER(ADC_HandleTypeDef* hadc)
         : adc_handle_(hadc)
    {

    }
    ~ADC_DRIVER();
    ADC_DRIVER(const ADC_DRIVER&) = delete;
    ADC_DRIVER& operator=(const ADC_DRIVER&) = delete;

    void Init();
    [[nodiscard]] uint16_t getRaw(size_t ch) const;
    [[nodiscard]] float getMapped(size_t ch, float outMin, float outMax) const;

};


#endif
