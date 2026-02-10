#ifndef SERIALPROTOCOL_H
#define SERIALPROTOCOL_H

#pragma once

#include "ProtocolBase.h"
#include <cstdio>
#include <cstdarg>

/**
 * @brief 串口协议类
 */
class SerialProtocol : public ProtocolBase
{
public:
    enum SerialMode
    {
        IT, // 中断
        DMA, // DMA
    };

    explicit SerialProtocol(UART_HandleTypeDef* _huart)
        : ProtocolBase(512), SerialHuart(_huart),rxBuffer(256)
    {
    }

    SerialProtocol(const SerialProtocol&) = delete;
    SerialProtocol& operator=(const SerialProtocol&) = delete;

    //=====================初始化函数============================
    void Init(SerialMode Mode);
    void InitRS485(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
    void SetRxMode();
    void RestartRx();

    //=====================功能函数============================
    void OnCommFailure(CommError err) override;

    //=====================发送函数============================
    bool RawSend(const uint8_t* pData, size_t len) override;
    void SendByte(uint8_t Byte);
    void SendBytes(const uint8_t* pData, size_t Size);
    void SendString(const char* string);
    void SendString(const std::string& string) override;
    void SendNumber(int number);
    void SendNumber(float number, int precision) override;

    //重定向Printf
    int fputc(int ch, FILE* f);
    void Printf(const char* format, ...);

    //=====================接收函数============================
    void HandleRxData(uint16_t len);
    void HandleRxData(uint8_t* buf, uint16_t len);

private:
    UART_HandleTypeDef* SerialHuart; // 串口句柄
    GPIO_TypeDef* Serial485GPIO = nullptr;
    uint16_t Serial485Pin = 0;
    std::vector<uint8_t> rxBuffer;
};


class RS485Guard
{
public:
    RS485Guard(UART_HandleTypeDef* huart, GPIO_TypeDef* gpio, uint16_t pin)
        : _huart(huart), _gpio(gpio), _pin(pin) {
        if (_gpio) {
            HAL_GPIO_WritePin(_gpio, _pin, GPIO_PIN_SET);
        }
    }

    ~RS485Guard() {
        if (_gpio) {
            uint32_t timeout = 5000;
            while (__HAL_UART_GET_FLAG(_huart, UART_FLAG_TC) == RESET && timeout-- > 0) {}
            HAL_GPIO_WritePin(_gpio, _pin, GPIO_PIN_RESET);
        }
    }
private:
    UART_HandleTypeDef* _huart;
    GPIO_TypeDef* _gpio;
    uint16_t _pin;
};



#endif // SERIALPROTOCOL_H
