#pragma once
#include <cstdint>

/**
 * @brief 极速数学查表库
 * @details 存放所有预计算的常量表，用于替换耗时的浮点数学运算
 */
namespace FastMath {
    // 256 级正弦表，幅值放大到 255 (避免浮点运算)
    extern const int16_t SinTable_256[256];
}