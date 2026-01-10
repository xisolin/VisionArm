#ifndef CTRL_STEP_FW_ENCODER_H
#define CTRL_STEP_FW_ENCODER_H

#pragma once

#include <cstdint>


class EncoderBase
{
public:
    typedef struct
    {
        uint16_t rawAngle;          // raw data   初始角度
        uint16_t rectifiedAngle;    // calibrated rawAngle data  校准以后角度
        bool rectifyValid;          //数据是否有效
    } AngleData_t;
    //存储当前角度初始化为0
    AngleData_t angleData{0};


    /*
     * Resolution is (2^14 = 16384), each state will use an uint16 data
     * as map, thus total need 32K-flash for calibration.
    */
    // 编码器的分辨率，定义为 2 的 14 次方
    const int32_t RESOLUTION = ((int32_t) ((0x00000001U) << 14));

    virtual ~EncoderBase()  = default;

    virtual bool Init() = 0;

    //更新并获取当前的原始角度值
    // 返回最新的原始角度 rawAngle
    // Get current rawAngle
    virtual uint16_t UpdateAngle() = 0;

    // 检查编码器是否已校准
    virtual bool IsCalibrated() = 0;


private:

};

#endif
