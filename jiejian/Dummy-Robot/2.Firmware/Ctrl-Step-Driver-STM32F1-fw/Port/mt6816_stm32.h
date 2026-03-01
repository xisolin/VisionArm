#pragma once

#include "main.h"
#include <cstdint>

class TB67H450_FOC {
    static constexpr int32_t MAX_CURRENT_MA_ = 2000;    // 电机最大允许相电流 (例如 2000mA)
    static constexpr uint32_t PWM_PERIOD_ARR_ = 4000;   // 定时器 ARR 值 (决定了最大占空比)

    // 256 级快速正弦表 (用于极速 FOC 矢量计算，幅值 0~255)
    static const int16_t FastSinTable[256];

    void SetPhaseA_Voltage(int32_t _voltageA_InPWM);
    void SetPhaseB_Voltage(int32_t _voltageB_InPWM);

public:
    explicit TB67H450_FOC(
        TIM_HandleTypeDef* htim_A, uint32_t channel_A,
        TIM_HandleTypeDef* htim_B, uint32_t channel_B,
        GPIO_TypeDef* _pAP, uint16_t _pinAP, GPIO_TypeDef* _pAM, uint16_t _pinAM,
        GPIO_TypeDef* _pBP, uint16_t _pinBP, GPIO_TypeDef* _pBM, uint16_t _pinBM
    );
    void Init();
    void Sleep(); // 关闭所有输出，释放电机
    void SetFocCurrentVector(uint8_t _electricalAngle_0_255, int32_t _targetCurrent_mA);

private:
    TIM_HandleTypeDef* htim_A_; uint32_t channel_A_;
    TIM_HandleTypeDef* htim_B_; uint32_t channel_B_;

    GPIO_TypeDef* port_AP_; uint16_t pin_AP_;
    GPIO_TypeDef* port_AM_; uint16_t pin_AM_;
    GPIO_TypeDef* port_BP_; uint16_t pin_BP_;
    GPIO_TypeDef* port_BM_; uint16_t pin_BM_;

};