//
// Created by 0209 on 2026/2/11.
//

#ifndef SERIALPORT_H
#define SERIALPORT_H

#include "SimpleShell.h"
#include "main.h"

class SerialPortDriver : public IShellPort {
public:
    explicit SerialPortDriver(UART_HandleTypeDef* huart) : _huart(huart) {}

    void Write(const char* data, int len) override {
        HAL_UART_Transmit(_huart, (uint8_t*)data, len, 100);
    }
    UART_HandleTypeDef* _huart;
};


class SerialShell : public SimpleShell {
public:
    explicit SerialShell(UART_HandleTypeDef* huart)
        : _internalDriver(huart), _rxByte(0),
          SimpleShell(&_internalDriver)
    {
    }

    void Init() {
        SimpleShell::Init(); // 1. 先调用父类的初始化(打印Banner等)
        StartRx();           // 2. 自动启动接收
    }

    void HandleRxInterrupt() {
        Input((char)_rxByte);
        // B. 重新开启中断
        StartRx();
    }

    UART_HandleTypeDef* GetHandle() {
        return _internalDriver._huart;
    }

private:
    SerialPortDriver _internalDriver;
    uint8_t _rxByte;
    void StartRx() {
        HAL_UART_Receive_IT(_internalDriver._huart, &_rxByte, 1);
    }
};


#endif //SERIALPORT_H


/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == Shell.GetHandle())
    {
        Shell.HandleRxInterrupt();
    }
}

 */