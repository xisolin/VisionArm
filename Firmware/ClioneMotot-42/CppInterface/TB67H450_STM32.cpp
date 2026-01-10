//
// Created by 0209 on 2026/1/10.
//

#include "TB67H450_STM32.h"



void TB67H450_STM32::InitGpio()
{

}

void TB67H450_STM32::InitPwm()
{
    // 启动 PWM 输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // A相电流控制
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // B相电流控制
}

void TB67H450_STM32::SetInputA(bool _statusAp, bool _statusAm)
{
    HAL_GPIO_WritePin(IN_AM_GPIO_Port, IN_AM_Pin, _statusAp ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN_AP_GPIO_Port, IN_AP_Pin, _statusAm ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void TB67H450_STM32::SetInputB(bool _statusBp, bool _statusBm)
{
    HAL_GPIO_WritePin(IN_BM_GPIO_Port, IN_BM_Pin, _statusBp ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN_BP_GPIO_Port, IN_BP_Pin, _statusBm ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void TB67H450_STM32::DacOutputVoltage(uint16_t _voltageA, uint16_t _voltageB)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, _voltageA);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, _voltageB);
}
