//
// Created by 0209 on 2026/2/10.
//

#ifndef SYSTEMMONITOR_H
#define SYSTEMMONITOR_H

#pragma once

#include "Cpphand.h"

class SystemMonitor {
public:
    enum class SystemState {
        BOOTING,    // 启动中
        NORMAL,     // 正常运行
        WARNING,    // 警告
        CRITICAL,   // 严重故障
        RECOVERY    // 故障恢复中
    };

    SystemMonitor(ProtocolBase& protocol, GPIO_TypeDef* ledPort, uint16_t ledPin)
        : _protocol(protocol), _ledPort(ledPort), _ledPin(ledPin) {}

    [[nodiscard]] SystemState GetState() const { return _currentState; }
    void Update(uint32_t timeoutMs = 200);

private:
    void HandleFeedback();

    uint32_t _lastBlinkTick = 0; // 用于控制闪烁频率
    ProtocolBase& _protocol;
    SystemState _currentState = SystemState::BOOTING;

    GPIO_TypeDef* _ledPort;
    uint16_t _ledPin;
};


#endif //SYSTEMMONITOR_H
