//
// Created by 0209 on 2026/2/10.
//

#ifndef SAFETYWATCHDOG_H
#define SAFETYWATCHDOG_H

#include <cstdint>

// 定义回调函数的函数指针类型，方便阅读
// 1. 硬件动作回调 (无参数，无返回值) -> 用于喂狗、复位
typedef void (*HardwareAction_t)(void);

// 2. 超时报警回调 (参数是丢失的任务掩码) -> 用于写黑匣子
typedef void (*TimeoutCallback_t)(uint32_t missingMask);

class SafetyWatchdog {
public:

    SafetyWatchdog(uint8_t taskCount, HardwareAction_t feedFunc, HardwareAction_t resetFunc);
    void CheckIn(uint8_t taskId);
    void TryFeed(TimeoutCallback_t onTimeout = nullptr);
    void ForceReset();
    uint32_t GetMissingMask() const;

private:
    uint32_t _targetMask;           // 目标掩码 (所有人都签到时的值)
    volatile uint32_t _checkInFlags;// 当前签到状态

    HardwareAction_t _feedFunc;     // 硬件喂狗函数
    HardwareAction_t _resetFunc;    // 硬件复位函数
};



#endif //SAFETYWATCHDOG_H
