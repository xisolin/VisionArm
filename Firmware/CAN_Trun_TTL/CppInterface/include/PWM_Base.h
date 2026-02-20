#pragma once

#include "Cpphand.h"

/*
 * 通用的PWM类
 */
class PWM_Base
{
protected:
    TIM_HandleTypeDef* htim;
    uint32_t channel;

public:
    PWM_Base(TIM_HandleTypeDef* _htim, uint32_t _channel) :
        htim(_htim), channel(_channel){}
    virtual ~PWM_Base() = default;
    // ===================== PWM初始化接口 =====================
    /*
     * 初始化普通 PWM 输出
     * 只启动 PWM，不使用中断或 DMA
     */
    void start(uint16_t CCR)
    {
        __HAL_TIM_SET_COMPARE(htim, channel, CCR);
        HAL_TIM_PWM_Start(htim, channel);
    }

    /*
     * 初始化 PWM 输出，可选择是否使用中断
     *
     * 参数:
     *   useInterrupt - true 启用中断模式，false 普通 PWM
     */
void startIT(uint16_t CCR)
    {
        __HAL_TIM_SET_COMPARE(htim, channel, CCR);
        HAL_TIM_PWM_Start_IT(htim, channel);
    }

    /*
     * 初始化 PWM 输出，使用 DMA 自动传输占空比数据
     *
     * 参数:
     *   buffer - 指向占空比数组的指针
     *   length - 数组长度（元素个数）
     */
    void startDMA(uint32_t* buffer, uint16_t length)
    {
        // DMA 模式通常不需要预设 CCR，因为 buffer 会立即覆盖它
        HAL_TIM_PWM_Start_DMA(htim, channel, buffer, length);
    }

    void stop()
    {
        HAL_TIM_PWM_Stop(htim, channel);
        HAL_TIM_PWM_Stop_IT(htim, channel);
        HAL_TIM_PWM_Stop_DMA(htim, channel);
    }
    // =======================================================


    // ===================== 参数设置接口 =====================
    /*
     * 设置占空比
     */
    void setDuty(uint16_t duty)
    {
        __HAL_TIM_SET_COMPARE(htim, channel, duty);
    }

    /*
     * 设置 ARR 寄存器（PWM 周期）
     * value = 自动重装载值，ARR+1 是定时器计数周期
     */
    void setARR(uint32_t value)
    {
        __HAL_TIM_SET_AUTORELOAD(htim, value);
    }

    /*
     * 设置 PSC 寄存器（预分频）
     * value = 预分频值，计数频率 = 定时器时钟 / (PSC+1)
     */
    void setPSC(uint32_t value)
    {
        __HAL_TIM_SET_PRESCALER(htim, value);
    }

    // ===================== 参数读取接口=====================

    /*
     * 获取当前占空比（CCR 值）
     */
    uint32_t getCCR() const
    {
        return __HAL_TIM_GET_COMPARE(htim, channel);
    }

    /*
     * 获取当前 ARR
     */
    uint32_t getARR() const
    {
        return __HAL_TIM_GET_AUTORELOAD(htim);
    }

    /*
     * 获取当前 PSC
     */
    uint32_t getPSC() const
    {
        return htim->Instance->PSC;
    }
};

/*
//关闭PWM波的中断
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) //中断回调
{
    static uint16_t i = 0;
    if (htim->Instance == TIM3) {
        i++;
        if (i >= 1)  //此处10为产生PWM的数量，可设置为变量实时改变
        {
            i = 0;
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
        }
    }
}
*/
