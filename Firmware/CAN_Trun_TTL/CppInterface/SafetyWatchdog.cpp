#include "SafetyWatchdog.h"

/**
 * @brief 构造函数
 * @param taskCount  任务总数
 * @param feedFunc   硬件喂狗函数指针
 * @param resetFunc  硬件复位函数指针
 */
SafetyWatchdog::SafetyWatchdog(uint8_t taskCount, HardwareAction_t feedFunc, HardwareAction_t resetFunc)
    : _feedFunc(feedFunc), _resetFunc(resetFunc), _checkInFlags(0)
{
    // 计算目标掩码
    // 如果 taskCount = 3，则 1<<3 = 1000(二进制)，减1得到 0111(二进制) = 7
    // 这样我们就有了一个全1的掩码
    if (taskCount >= 32) {
        _targetMask = 0xFFFFFFFF; // 防止移位溢出
    } else {
        _targetMask = (1UL << taskCount) - 1;
    }
}

/**
 * @brief 任务签到
 * @param taskId 任务ID (0 到 taskCount-1)
 */
void SafetyWatchdog::CheckIn(uint8_t taskId)
{
    // 保护：防止 taskId 超出范围
    if ((1UL << taskId) & _targetMask) {
        // 将对应位置 1
        _checkInFlags |= (1UL << taskId);
    }
}

/**
 * @brief 尝试喂狗
 * @details 检查是否所有人都签到了
 * @param onTimeout 如果有人没签到，调用这个回调函数记录错误 (可选)
 */
void SafetyWatchdog::TryFeed(TimeoutCallback_t onTimeout)
{
    // 检查当前状态是否等于目标状态 (是不是所有位都置1了)
    if (_checkInFlags == _targetMask) {
        // 1. 大家都签到了 -> 喂狗
        if (_feedFunc) {
            _feedFunc();
        }
        
        // 2. 清除签到表，开启下一轮
        _checkInFlags = 0;
    }
    else {
        // 异常：有人没签到！
        
        // 1. 如果提供了报警回调，先报警 (写黑匣子)
        if (onTimeout) {
            uint32_t missing = (~_checkInFlags) & _targetMask;
            onTimeout(missing);
        }

        // 2. 强制复位
        ForceReset();
    }
}

/**
 * @brief 强制复位
 */
void SafetyWatchdog::ForceReset()
{
    if (_resetFunc) {
        _resetFunc();
    }
}

/**
 * @brief 获取缺席名单 (调试用)
 */
uint32_t SafetyWatchdog::GetMissingMask() const
{
    return (~_checkInFlags) & _targetMask;
}