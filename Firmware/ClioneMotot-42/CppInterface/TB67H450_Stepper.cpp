/**
 * @file    TB67H450_Stepper.cpp
 * @brief   专为 STM32 极速优化的 TB67H450 矢量驱动底层代码
 */

#include "TB67H450_Stepper.h"
#include "FastMathLUT.h"

// ===================================================
// 🌟 硬件映射区：直接把引脚和端口焊死在这里！
// 请根据你的实际原理图修改这里的端口和引脚号
// ===================================================
#define PORT_A_PHASE GPIOA
#define PIN_AP       GPIO_PIN_5
#define PIN_AM       GPIO_PIN_4

#define PORT_B_PHASE GPIOA
#define PIN_BP       GPIO_PIN_3
#define PIN_BM       GPIO_PIN_2


TB67H450_Stepper::TB67H450_Stepper(
    TIM_HandleTypeDef* htim_A, uint32_t channel_A,
    TIM_HandleTypeDef* htim_B, uint32_t channel_B)
    : htim_A_(htim_A), channel_A_(channel_A),
      htim_B_(htim_B), channel_B_(channel_B)
{}

void TB67H450_Stepper::Init() {
    // 开启定时器 PWM 输出
    HAL_TIM_PWM_Start(htim_A_, channel_A_);
    HAL_TIM_PWM_Start(htim_B_, channel_B_);
    Sleep();
}

void TB67H450_Stepper::Sleep() {
    SetPhaseA_Voltage(0);
    SetPhaseB_Voltage(0);
}

void TB67H450_Stepper::SetPhaseA_Voltage(int32_t _voltageA_InPWM) {
    if (_voltageA_InPWM > 0) {
        PORT_A_PHASE->BSRR = PIN_AP;
        PORT_A_PHASE->BRR  = PIN_AM;
        __HAL_TIM_SET_COMPARE(htim_A_, channel_A_, _voltageA_InPWM);
    } else if (_voltageA_InPWM < 0) {
        PORT_A_PHASE->BRR  = PIN_AP;
        PORT_A_PHASE->BSRR = PIN_AM;
        __HAL_TIM_SET_COMPARE(htim_A_, channel_A_, -_voltageA_InPWM);
    } else {
        PORT_A_PHASE->BRR = PIN_AP;
        PORT_A_PHASE->BRR = PIN_AM;
        __HAL_TIM_SET_COMPARE(htim_A_, channel_A_, 0);
    }
}

void TB67H450_Stepper::SetPhaseB_Voltage(int32_t _voltageB_InPWM) {
    if (_voltageB_InPWM > 0) {
        PORT_B_PHASE->BSRR = PIN_BP;
        PORT_B_PHASE->BRR  = PIN_BM;
        __HAL_TIM_SET_COMPARE(htim_B_, channel_B_, _voltageB_InPWM);
    } else if (_voltageB_InPWM < 0) {
        PORT_B_PHASE->BRR  = PIN_BP;
        PORT_B_PHASE->BSRR = PIN_BM;
        __HAL_TIM_SET_COMPARE(htim_B_, channel_B_, -_voltageB_InPWM);
    } else {
        PORT_B_PHASE->BRR = PIN_BP;
        PORT_B_PHASE->BRR = PIN_BM;
        __HAL_TIM_SET_COMPARE(htim_B_, channel_B_, 0);
    }
}


void TB67H450_Stepper::SetFocCurrentVector(uint8_t _electricalAngle_0_255, int32_t _targetCurrent_mA) {
    // 1. 电流限幅保护 (非常重要，防止烧坏)
    if (_targetCurrent_mA > MAX_CURRENT_MA_) _targetCurrent_mA = MAX_CURRENT_MA_;
    if (_targetCurrent_mA < -MAX_CURRENT_MA_) _targetCurrent_mA = -MAX_CURRENT_MA_;

    // 2. 将毫安(mA) 映射到 PWM 占空比
    int32_t pwm_amplitude = (_targetCurrent_mA * PWM_PERIOD_ARR_) / MAX_CURRENT_MA_;

    // 3. 从 FastMath 查表获取正余弦值
    int32_t sin_val = FastMath::SinTable_256[_electricalAngle_0_255];
    int32_t cos_val = FastMath::SinTable_256[(uint8_t)(_electricalAngle_0_255 + 64)];

    // 4. 计算两相实际输出 PWM
    int32_t phaseA_pwm = (pwm_amplitude * sin_val) / 255;
    int32_t phaseB_pwm = (pwm_amplitude * cos_val) / 255;

    // 5. 极速打入底层
    SetPhaseA_Voltage(phaseA_pwm);
    SetPhaseB_Voltage(phaseB_pwm);
}