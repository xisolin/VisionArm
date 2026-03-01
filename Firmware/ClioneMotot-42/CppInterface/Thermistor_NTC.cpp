#include "Thermistor_NTC.h"

/**
 * @brief   构造函数：初始化 NTC 运行期配置与滤波器实例
 * @param r_fixed             下侧分压电阻的实际精确阻值，单位：欧姆
 * @param temp_offset         最终输出温度的线性校准偏移量，单位：摄氏度
 * @param filter_window_size  分配给内部滤波器的窗口大小（供滑动平均/中位值使用）
 */
Thermistor_NTC::Thermistor_NTC(float r_fixed, float temp_offset, uint8_t filter_window_size)
    : r_fixed_(r_fixed),
      temp_offset_(temp_offset),
      filter_(filter_window_size)
{}

/**
 * @brief   核心换算：将 12 位 ADC 原始采样值转换为 NTC 瞬时阻值
 * @note    硬件拓扑假定为：上侧 NTC 接 VCC，下侧 R_fixed 接 GND。
 * 内部包含了对边界异常值（0 和满量程）的安全防护，防止单片机触发除零异常。
 * @param adc_value    外部传入的 12 位 ADC 采样值 (0 ~ 4095)
 * @return float       计算得出的 NTC 实时阻值，单位：欧姆
 */
float Thermistor_NTC::ADC2Resistance(uint32_t adc_value) const {
    if (adc_value == 0) return INFINITY;
    if (adc_value >= ADC_MAX_RAW_) adc_value = ADC_SAFE_MAX_;
    return r_fixed_ * ((ADC_MAX_F_ - static_cast<float>(adc_value)) / static_cast<float>(adc_value));
}

/**
 * @brief   核心换算：根据 NTC 瞬时阻值计算对应的物理温度
 * @note    基于 Steinhart-Hart 热力学方程 (B 参数简化版) 进行推导。
 * @param r_ntc        当前 NTC 的实际物理阻值，单位：欧姆
 * @return float       计算出的环境温度，单位：摄氏度
 */
float Thermistor_NTC::Resistance2Temperature(float r_ntc) const {
    if (!(r_ntc > 0.0f)) return ABS_ZERO_C_;
    float invT = (1.0f / B_CONST_) * logf(r_ntc / R25_) + (1.0f / T25_K_);
    float T_K = 1.0f / invT;
    return (T_K - KELVIN_OFFSET_) + temp_offset_;
}

/**
 * @brief   数据更新总控引擎：输入原始 ADC 数据，获取平滑温度
 * @details 将原始数字量转换为阻值，再转为瞬时温度，最后通过卡尔曼滤波器压制高频干扰噪声。
 * @param raw_adc_value 外部测得的最新 12 位 ADC 原始值
 * @return float        经过一阶低通滤波 (IIR)处理后的最优温度估计值，单位：摄氏度
 */
float Thermistor_NTC::Update(uint16_t raw_adc_value) {
    float current_r = ADC2Resistance(raw_adc_value);
    float current_temp = Resistance2Temperature(current_r);
    return filter_.IIR(current_temp, 0.05f);
}