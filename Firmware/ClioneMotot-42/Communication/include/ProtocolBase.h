#ifndef PROTOCOLBASE_H
#define PROTOCOLBASE_H

#pragma once
#include <vector>
#include <array>
#include <cstring>
#include <string>
#include "stm32f1xx_hal.h"


class ProtocolBase
{
public:
    //=====================包头包尾============================
    static constexpr uint8_t FRAME_HEADER1 = 0xAA; // 帧头1
    static constexpr uint8_t FRAME_HEADER2 = 0x55; // 帧头2
    static constexpr uint8_t FRAME_TAIL1 = 0x55; // 帧尾1
    static constexpr uint8_t FRAME_TAIL2 = 0xAA; // 帧尾2
    //===================协议指令定义===========================
    static constexpr uint8_t CMD_ACK_ID = 0xFE;
    //===================== 应答状态码 (AckStatus) =============
    enum class AckStatus : uint8_t {
        OK              = 0x00, // 成功 / 已执行
        ERROR           = 0x01, // 通用错误
        BUSY            = 0x02, // 系统忙
        UNKNOWN_CMD     = 0x03, // 不认识的指令 ID
        PARAM_ERROR     = 0x04, // 参数超出范围
        CRC_FAIL        = 0x05, // 校验失败
        HARDWARE_FAULT  = 0x06  // 硬件故障
    };
    //=====================异常状态============================
    enum class CommError : uint8_t {
        NONE = 0,
        TIMEOUT,            // 通讯超时（心跳丢失）
        CRC_ERROR,          // CRC 校验错误
        BUFFER_OVERFLOW,    // 环形缓冲区溢出
        FRAME_FORMAT_ERROR, // 帧格式错误（帧头对但帧尾/长度不对）
        HARDWARE_FAILURE    // 底层驱动返回错误（如串口 ORE/NE）
    };
    [[nodiscard]] uint32_t GetErrorCount() const { return errorCount; }
    void ResetErrorCount() { errorCount = 0; }
    //=====================初始化函数===========================
    explicit ProtocolBase(size_t bufferSize);
    virtual ~ProtocolBase() = default;
    //=====================功能函数============================
    static uint16_t CRC16_Modbus(const uint8_t* data, uint16_t len);
    uint16_t CRC16_Modbus_Ring(size_t startIdx, size_t len);
    [[nodiscard]] bool IsCommAlive(uint32_t timeoutMs = 1000) const {
        return (HAL_GetTick() - lastActivityTick) < timeoutMs;
    }
    template <typename T>
    static T ParseStruct(const uint8_t* buffer, size_t len);
    //=====================发送函数============================
    void SendACK(uint8_t targetCmdId, AckStatus status);

    void SendWithCRC16(const std::vector<uint8_t>& data);
    virtual void SendString(const std::string& str);
    virtual void SendNumber(float number, int precision);

    template <typename T, size_t N>
    void SendArrayWithCRC16(const std::array<T, N>& arr);

    //=====================接收函数============================
    size_t ReceiveRaw(uint8_t* pOut, size_t maxLen);
    bool Receive(std::vector<uint8_t>& outPayload);
    template <size_t N>
    bool Receive(std::array<uint8_t, N>& outArray);

    //=====================异常函数============================

    void HandleHardwareError(CommError err);
    virtual void OnCommFailure(CommError err) {}

protected:
    virtual bool RawSend(const uint8_t* pData, size_t len) = 0;
    virtual void RawSend(const std::vector<uint8_t>& data)
    {
        RawSend(data.data(), data.size());
    }
    void SafeRawSend(const uint8_t* pData, size_t len);

    void WriteToRingBuffer(const uint8_t* data, size_t len);

private:
    std::vector<uint8_t> ProcBuffer;                //写入缓存区
    volatile size_t rxWriteIndex;                   //写指针
    volatile size_t rxReadIndex;                    //读指针
    static constexpr size_t kMinFrameLength = 7;    //最小包长

    uint32_t errorCount = 0;                        //异常次数
    CommError lastError = CommError::NONE;          //异常状态
    uint32_t lastActivityTick = 0;                  //超时检测
    uint32_t lastByteReceivedTick = 0;              //记录最后一次收到字节的时间

    size_t FindFrameHeader(size_t start, size_t size);
    uint16_t RingBuffer_Size();
};


class ScopedACK {
public:
    // 构造函数：记录必要的上下文，默认状态设为 OK
    ScopedACK(ProtocolBase& protocol, uint8_t cmdId)
        : _protocol(protocol), _cmdId(cmdId), _status(ProtocolBase::AckStatus::OK) {}

    // 析构函数：离开作用域时自动发送 ACK
    ~ScopedACK() {
        if (_enable) {
            _protocol.SendACK(_cmdId, _status);
        }
    }

    // 如果执行过程中发现错误，调用这个改状态
    void SetStatus(ProtocolBase::AckStatus status) {
        _status = status;
    }
    // 如果某些特殊指令不需要回 ACK（比如静默指令），可以手动关闭
    void Disable() {
        _enable = false;
    }

private:
    ProtocolBase& _protocol;
    uint8_t _cmdId;
    ProtocolBase::AckStatus _status;
    bool _enable = true;
};

#include "ProtocolBase_Impl.hpp"

#endif
