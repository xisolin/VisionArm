#include "filter.h"


// ==================== 构造函数 ====================
// 构造函数用来初始化滤波器对象
// size: 滤波窗口长度（滑动平均和中位值用）
// 初始化索引、计数、上一次值、卡尔曼估计值
Filter::Filter(uint8_t size)
    : windowSize(size),  // 滤波窗口大小
      index(0),          // 当前写入索引初始化为0
      count(0),          // 当前有效数据数量初始化为0
      lastValue(0),      // 上一次滤波输出值初始化为0
      kalmanX(0),        // 卡尔曼估计值初始化为0
      kalmanP(1)         // 卡尔曼估计协方差初始化为1
{
    buf.fill(0.0f);     // 将缓存数组初始化为0
}

// ==================== 滑动平均滤波 ====================
// 原理：保存最近 windowSize 个采样值，计算平均值
// 输入 value：新的ADC采样值
// 返回：滑动平均后的值
float Filter::MovingAverage(float value) {
    buf[index] = value;                  // 将新采样值写入缓存
    index = (index + 1) % windowSize;    // 环形缓存索引，超过窗口长度回到0
    if (count < windowSize) count++;     // 未填满时，增加有效计数

    float sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += buf[i]; // 累加有效缓存值
    lastValue = sum / count;             // 计算平均值
    return lastValue;                    // 返回滤波结果
}

// ==================== 中位值滤波 ====================
// 原理：取一段时间内的采样值排序后取中间值
// 优点：对脉冲干扰、异常点特别有效
float Filter::Median(float value) {
    buf[index] = value;                  // 保存新采样值
    index = (index + 1) % windowSize;    // 环形缓存
    if (count < windowSize) count++;     // 增加有效数据计数

    std::array<float, 32> tmp{};         // 临时数组，用于排序
    std::copy(buf.begin(), buf.begin() + count, tmp.begin()); // 拷贝有效数据
    std::sort(tmp.begin(), tmp.begin() + count);             // 排序

    lastValue = tmp[count / 2];          // 取中间值作为滤波结果
    return lastValue;
}

// ==================== 限幅滤波 ====================
// 原理：限制信号变化幅度，防止突变干扰
// value：当前采样值
// limit：最大允许跳变值
float Filter::Limit(float value, float limit) {
    if ((value - lastValue) > limit)      // 当前值比上一次值增幅过大
        value = lastValue + limit;        // 限制为最大跳变
    else if ((lastValue - value) > limit) // 当前值下降幅度过大
        value = lastValue - limit;        // 限制为最大跳变

    lastValue = value;                     // 更新lastValue
    return lastValue;                      // 返回滤波结果
}

// ==================== 一阶IIR低通滤波 ====================
// 原理：利用指数加权平均平滑信号
// alpha：权重系数(0~1)，越小越平滑，响应越慢
float Filter::IIR(float value, float alpha) {
    lastValue = alpha * value + (1.0f - alpha) * lastValue; // IIR公式
    return lastValue;                                       // 返回滤波结果
}

// ==================== 简化卡尔曼滤波 ====================
// 原理：利用卡尔曼滤波结合系统预测与测量值
// value：当前采样值
// Q：过程噪声协方差，R：测量噪声协方差
float Filter::Kalman(float value, float Q, float R) {
    // -------- 预测步骤 --------
    kalmanP += Q;                     // 更新估计协方差

    // -------- 更新增益 --------
    float K = kalmanP / (kalmanP + R); // 卡尔曼增益

    // -------- 更新估计 --------
    kalmanX += K * (value - kalmanX);  // 利用新测量值修正估计

    // -------- 更新协方差 --------
    kalmanP *= (1 - K);               // 更新协方差

    return kalmanX;                   // 返回卡尔曼滤波后的估计值
}
