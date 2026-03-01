#ifndef FILTER_H
#define FILTER_H

#include <array>
#include <algorithm>
#include <cstdint>
#include <cstring>

class Filter {
public:
    // 构造函数，size是滤波窗口长度
    Filter(uint8_t size = 10);

    // 滑动平均滤波
    float MovingAverage(float value);

    // 中位值滤波
    float Median(float value);

    // 限幅滤波（如果值变化超过limit，则限制变化）
    float Limit(float value, float limit);

    // 一阶IIR低通滤波（alpha: 0~1, 越小越平滑）
    float IIR(float value, float alpha);

    // 卡尔曼滤波
    float Kalman(float value, float Q = 0.01f, float R = 0.1f);

private:
    uint8_t windowSize;
    std::array<float, 32> buf;  // 最多32点缓存
    uint8_t index;
    uint8_t count;

    float lastValue;  // 上一次滤波输出（IIR/Limit用）
    float kalmanX;    // 卡尔曼估计值
    float kalmanP;    // 卡尔曼协方差
};
#endif //FILTER_H
