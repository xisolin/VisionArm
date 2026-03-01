#pragma once

#include "main.h"
#include <cstdint>

class TB67H450_Stepper {
private:
    // ==========================================
    // 🌟 编译期常量区
    // ==========================================
    static constexpr int32_t MAX_CURRENT_MA_ = 2000;  // 电机最大允许相电流 (毫安)
    static constexpr uint32_t PWM_PERIOD_ARR_ = 3600 - 1; // 定时器 ARR 的最大值 (根据你的 CubeMX 配置修改)

    // 只保留定时器资源，抛弃所有 GPIO 变量以节省 RAM
    TIM_HandleTypeDef* htim_A_; uint32_t channel_A_;
    TIM_HandleTypeDef* htim_B_; uint32_t channel_B_;

    // 底层寄存器极速操作接口
    void SetPhaseA_Voltage(int32_t _voltageA_InPWM);
    void SetPhaseB_Voltage(int32_t _voltageB_InPWM);

public:
    // 构造函数：只需传入 PWM 定时器资源
    explicit TB67H450_Stepper(
        TIM_HandleTypeDef* htim_A, uint32_t channel_A,
        TIM_HandleTypeDef* htim_B, uint32_t channel_B
    );

    void Init();

    // 释放电机 (高阻态)
    void Sleep();

    /**
     * @brief 🌟 核心 FOC 矢量注入函数 (替代原来的 Step)
     * @param _electricalAngle_0_255  电角度 (0~255 对应一整个电周期 360°)
     * @param _targetCurrent_mA       目标矢量电流大小 (毫安)
     */
    void SetFocCurrentVector(uint8_t _electricalAngle_0_255, int32_t _targetCurrent_mA);
};