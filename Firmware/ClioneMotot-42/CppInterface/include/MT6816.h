#ifndef MT6816_H
#define MT6816_H

#pragma once
#include "Cpphand.h"

class MT6816 {
public:
    MT6816(SPI_HandleTypeDef* spi, GPIO_TypeDef* cs_port, uint16_t cs_pin);
    // 初始化传感器
    bool Init();
    // 读取 14 位原始数据 (0 ~ 16383)
    uint16_t GetRawAngle();
    // 读取转换后的角度 (0.0 ~ 360.0 度)
    float GetAngleDegrees();
private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
};



#endif //MT6816_H
