#include "TB67H450_Stepper.h"


/**
     * @brief 构造函数
     * @param _pwmA  控制 A 相 VREF 的 PWM 对象
     * @param _pwmB  控制 B 相 VREF 的 PWM 对象
     * @param _pAP   A+ GPIO 端口
     * @param _pinAP A+ GPIO 引脚
     * @param _pAM   A- GPIO 端口
     * @param _pinAM A- GPIO 引脚
     * @param _pBP   B+ GPIO 端口
     * @param _pinBP B+ GPIO 引脚
     * @param _pBM   B- GPIO 端口
     * @param _pinBM B- GPIO 引脚
     */
TB67H450_Stepper::TB67H450_Stepper(
    TIM_HandleTypeDef* htim_A, uint32_t channel_A, // A相
    TIM_HandleTypeDef* htim_B, uint32_t channel_B, // B相
    GPIO_TypeDef* _pAP, uint16_t _pinAP,
    GPIO_TypeDef* _pAM, uint16_t _pinAM,
    GPIO_TypeDef* _pBP, uint16_t _pinBP,
    GPIO_TypeDef* _pBM, uint16_t _pinBM
)
: pwm_VrefA(htim_A, channel_A),
  pwm_VrefB(htim_B, channel_B),
  port_AP(_pAP), pin_AP(_pinAP),
  port_AM(_pAM), pin_AM(_pinAM),
  port_BP(_pBP), pin_BP(_pinBP),
  port_BM(_pBM), pin_BM(_pinBM)
{
    step_index = 0;
    current_torque = 0;
}

// 内部辅助函数：设置 A 相电平状态
// val: 1=正向电流, -1=反向电流, 0=关闭
void TB67H450_Stepper::SetPhaseA(int val)
{
    if (val > 0) {
        // A+ High, A- Low
        HAL_GPIO_WritePin(port_AP, pin_AP, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port_AM, pin_AM, GPIO_PIN_RESET);
    } else if (val < 0) {
        // A+ Low, A- High
        HAL_GPIO_WritePin(port_AP, pin_AP, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port_AM, pin_AM, GPIO_PIN_SET);
    } else {
        // 全部拉低 (高阻/滑行)
        HAL_GPIO_WritePin(port_AP, pin_AP, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port_AM, pin_AM, GPIO_PIN_RESET);
    }
}

// 内部辅助函数：设置 B 相电平状态
void TB67H450_Stepper::SetPhaseB(int val)
{
    if (val > 0) {
        // B+ High, B- Low
        HAL_GPIO_WritePin(port_BP, pin_BP, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port_BM, pin_BM, GPIO_PIN_RESET);
    } else if (val < 0) {
        // B+ Low, B- High
        HAL_GPIO_WritePin(port_BP, pin_BP, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port_BM, pin_BM, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(port_BP, pin_BP, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port_BM, pin_BM, GPIO_PIN_RESET);
    }
}

/**
 * @brief 初始化电机
 * @param initialTorque 初始力矩 (PWM占空比，0~ARR)
 * 建议从 30%-50% 开始，太高电机发烫
 */
void TB67H450_Stepper::Init(uint16_t initialTorque)
{
    current_torque = initialTorque;

    // 1. 启动 PWM (输出 VREF 电压)
    pwm_VrefA.start(current_torque);
    pwm_VrefB.start(current_torque);

    // 2. 默认让电机处于释放状态 (不通电)
    Release();
}

/**
 * @brief 设置力矩 (电流限制)
 * @param torque PWM占空比。值越大，VREF越高，电流越大，劲儿越大。
 */
void TB67H450_Stepper::SetTorque(uint16_t torque)
{
    current_torque = torque;
    pwm_VrefA.setDuty(torque);
    pwm_VrefB.setDuty(torque);
}

/**
 * @brief 走一步
 * @param dir  1: 正转, -1: 反转
 */
void TB67H450_Stepper::Step(int dir)
{
    // 1. 更新步数索引 (0 -> 1 -> 2 -> 3 -> 0)
    if (dir > 0) step_index++;
    else         step_index--;

    // 环形处理
    if (step_index > 3) step_index = 0;
    if (step_index < 0) step_index = 3;

    // 2. 双相励磁逻辑 (Two-Phase On)
    switch (step_index) {
    case 0:
        SetPhaseA(1);  SetPhaseB(1);  // A+, B+
        break;
    case 1:
        SetPhaseA(-1); SetPhaseB(1);  // A-, B+
        break;
    case 2:
        SetPhaseA(-1); SetPhaseB(-1); // A-, B-
        break;
    case 3:
        SetPhaseA(1);  SetPhaseB(-1); // A+, B-
        break;
    }
}

/**
 * @brief 释放电机 (脱力)
 * 电机线圈断电，轴可以手拧动，发热最低
 */
void TB67H450_Stepper::Release()
{
    SetPhaseA(0);
    SetPhaseB(0);
}

/**
 * @brief 锁定电机 (刹车)
 * 保持当前线圈通电，轴锁死，会有发热
 */
void TB67H450_Stepper::Lock()
{
    SetPhaseA(1);
    SetPhaseB(1);
}