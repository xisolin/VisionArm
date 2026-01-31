#pragma once

#include "main.h"
#include "PWM_Base.h"

class TB67H450_Stepper {
private:
    // 直接包含对象，生命周期随 Stepper 管理
    PWM_Base pwm_VrefA;
    PWM_Base pwm_VrefB;

    // GPIO 控制
    GPIO_TypeDef* port_AP; uint16_t pin_AP;
    GPIO_TypeDef* port_AM; uint16_t pin_AM;
    GPIO_TypeDef* port_BP; uint16_t pin_BP;
    GPIO_TypeDef* port_BM; uint16_t pin_BM;

    int step_index;
    uint16_t current_torque;

    void SetPhaseA(int val);
    void SetPhaseB(int val);

public:
    TB67H450_Stepper(
        TIM_HandleTypeDef* htim_A, uint32_t channel_A, // A相资源
        TIM_HandleTypeDef* htim_B, uint32_t channel_B, // B相资源
        GPIO_TypeDef* _pAP, uint16_t _pinAP,
        GPIO_TypeDef* _pAM, uint16_t _pinAM,
        GPIO_TypeDef* _pBP, uint16_t _pinBP,
        GPIO_TypeDef* _pBM, uint16_t _pinBM
    );

    void Init(uint16_t initialTorque = 500);
    void SetTorque(uint16_t torque);
    void Step(int dir);
    void Release();
    void Lock();
};