//
// Created by 0209 on 2026/2/10.
//

#include "SystemMonitor.h"

/**
 * @brief  [核心监护逻辑] 周期性检查系统体征并更新状态机
 * @details 该函数应在 FreeRTOS 任务中周期性调用（建议 20ms-50ms）。
 * 它实现了以下分级故障诊断逻辑：
 * 1. **最高优先级**：检查通讯心跳。如果超时，强制进入 CRITICAL 状态。
 * 2. **次高优先级**：检查通讯误码率。如果错误累计 > 50，进入 WARNING 状态。
 * 3. **恢复逻辑**：如果从故障中恢复，先进入 RECOVERY 过渡态，最后稳定在 NORMAL。
 *
 * @param  timeoutMs : 通讯断连判定阈值 (默认 200ms)。
 * 超过该时间未收到合法包，视为通讯中断。
 * @note
 * - 只有在状态稳定回落到 NORMAL 时，才会自动清除协议栈的错误计数 (@ref ResetErrorCount)。
 * - 该函数内部调用了 @ref HandleFeedback 进行 UI 更新。
 */
void SystemMonitor::Update(uint32_t timeoutMs)
{
    bool isAlive = _protocol.IsCommAlive(timeoutMs);

    if (!isAlive) {
        _currentState = SystemState::CRITICAL;
    }
    else {
        uint32_t errs = _protocol.GetErrorCount();

        if (errs > 50) {
            _currentState = SystemState::WARNING;
        }
        else {
            if (_currentState == SystemState::CRITICAL) {
                _currentState = SystemState::RECOVERY;
            } else {
                _currentState = SystemState::NORMAL;
                _protocol.ResetErrorCount();
            }
        }
    }
    HandleFeedback();
}

/**
 * @brief  [视觉反馈] 根据当前状态控制 LED 指示灯
 * @details 采用非阻塞式时间戳比较 (@ref HAL_GetTick)，确保不会阻塞主控制循环。
 * 指示灯定义：
 * - **NORMAL** : 常亮 (GPIO High)
 * - **CRITICAL** : 快闪 (100ms 周期, 5Hz) -> 提示紧急停机
 * - **WARNING** : 慢闪 (500ms 周期, 1Hz) -> 提示亚健康
 * - **RECOVERY** : (默认走 switch 穿透) 保持上一状态或常亮
 *
 * @note   此函数依赖系统滴答定时器，请确保 HAL_IncTick() 正常运行。
 */
void SystemMonitor::HandleFeedback()
{
    uint32_t now = HAL_GetTick();

    switch (_currentState) {
    case SystemState::NORMAL:
    case SystemState::RECOVERY:
        HAL_GPIO_WritePin(_ledPort, _ledPin, GPIO_PIN_RESET);
        break;

    case SystemState::CRITICAL:
        if (now - _lastBlinkTick > 100) {
            HAL_GPIO_TogglePin(_ledPort, _ledPin);
            _lastBlinkTick = now;
        }
        break;

    case SystemState::WARNING:
        if (now - _lastBlinkTick > 500) {
            HAL_GPIO_TogglePin(_ledPort, _ledPin);
            _lastBlinkTick = now;
        }
        break;

    default:
        break;
    }
}
