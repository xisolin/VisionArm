#include "ADC_Driver.h"

/*
 * 构造函数绑定硬件ADC句柄
 */

ADC_DRIVER::~ADC_DRIVER()
{
    // 良好的习惯：先判空再操作，虽然 HAL 内部可能也有检查
    if (adc_handle_) {
        HAL_ADC_Stop_DMA(adc_handle_);
    }
}

/*
 * 初始化
 */
void ADC_DRIVER::Init() {
    if (!adc_handle_) return;

    HAL_ADCEx_Calibration_Start(adc_handle_);

    // 使用 .data() 获取指针，reinterpret_cast 转换类型
    HAL_ADC_Start_DMA(
        adc_handle_,
        reinterpret_cast<uint32_t*>(adc_buffer_.data()),
        adc_buffer_.size()
    );
}

/*
 * 返回原始数值
 */
uint16_t ADC_DRIVER::getRaw(size_t ch) const
{
    if (ch >= adc_buffer_.size()) {
        return 0;
    }
    return adc_buffer_[ch];
}

/*
 * @brief 获取映射后的数值（将 ADC 原始值映射到任意区间）
 * @param ch          ADC 通道号
 * @param outMin      映射后最小值
 * @param outMax      映射后最大值
 * @return            映射后的值，超出范围则返回0
 */
float ADC_DRIVER::getMapped(size_t ch, float outMin, float outMax) const
{
    uint16_t raw = getRaw(ch);

    // 映射公式：y = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
    return (static_cast<float>(raw) / ADC_MAX_VALUE) * (outMax - outMin) + outMin;
}