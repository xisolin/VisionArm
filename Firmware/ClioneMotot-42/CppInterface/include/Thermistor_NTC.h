#pragma once
#include <cmath>
#include "filter.h"

class Thermistor_NTC {
    // 1. 物理与热力学常量
    static constexpr float R25_ = 10000.0f;                      // NTC 标称阻值
    static constexpr float B_CONST_ = 3950.0f;                   // NTC B常数
    static constexpr float KELVIN_OFFSET_ = 273.15f;             // 摄氏度转开尔文的偏移量
    static constexpr float T25_K_ = 25.0f + KELVIN_OFFSET_;      // 25℃对应的开尔文温度
    static constexpr float ABS_ZERO_C_ = -273.15f;               // 绝对零度 (摄氏)

    // 2. 硬件 ADC 常量
    static constexpr uint32_t ADC_MAX_RAW_ = 4095;               // 12位 ADC 最大原始值
    static constexpr uint32_t ADC_SAFE_MAX_ = 4094;              // 防止除零的安全最大值
    static constexpr float ADC_MAX_F_ = 4095.0f;                 // 浮点型满量程值，用于分压计算

    // 3. 滤波算法默认调参常量
    static constexpr float KALMAN_Q_ = 0.01f;                    // 卡尔曼过程噪声
    static constexpr float KALMAN_R_ = 0.1f;                     // 卡尔曼测量噪声

public:
    explicit Thermistor_NTC(float r_fixed = 10000.0f,
                            float temp_offset = 0.0f,
                            uint8_t filter_window_size = 10);

    float Update(uint16_t raw_adc_value);

private:

    const float r_fixed_;        // 具体硬件分压电阻
    const float temp_offset_;    // 软件标定偏移

    // 组合的滤波器对象
    Filter filter_;

    // 内部私有方法
    [[nodiscard]] float ADC2Resistance(uint32_t adc_value) const;
    [[nodiscard]] float Resistance2Temperature(float r_ntc) const;
};