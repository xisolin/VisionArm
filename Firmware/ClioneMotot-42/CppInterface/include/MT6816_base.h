#ifndef CTRL_STEP_FW_MT6816_H
#define CTRL_STEP_FW_MT6816_H

#pragma once

#include "Encoder_base.h"
#include <cstdint>

class MT6816Base : public EncoderBase
{
public:
    explicit MT6816Base(uint16_t* _quickCaliDataPtr) :
        spiRawData(SpiRawData_t{0}), // 初始化 SPI 原始数据结构体
        quickCaliDataPtr(_quickCaliDataPtr) // 初始化快速校准数据指针

    {
    }


    bool Init() override;
    uint16_t UpdateAngle() override; // Get current rawAngle (rad)更新并获取当前原始角度值。
    bool IsCalibrated() override;

private:
    typedef struct
    {
        uint16_t rawData; // SPI raw 16bits data    SPI 原始 16 位数据
        uint16_t rawAngle; // 14bits rawAngle in rawData   14 位角度值（编码器实际角度）
        bool noMagFlag; // 无磁信号标志
        bool checksumFlag; // 校验标志
    } SpiRawData_t;


    SpiRawData_t spiRawData;
    uint16_t* quickCaliDataPtr; // 快速校准数据指针
    uint16_t dataTx[2] = {0}; // SPI 发送缓冲
    uint16_t dataRx[2] = {0}; // SPI 接收缓冲
    uint8_t hCount; // 奇偶校验


    /***** Port Specified Implements *****/
    virtual void SpiInit() = 0;

    virtual uint16_t SpiTransmitAndRead16Bits(uint16_t _dataTx) = 0;
};

#endif
