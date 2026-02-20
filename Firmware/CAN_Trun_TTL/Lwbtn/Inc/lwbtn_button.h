#ifndef LWBTN_BUTTON_H
#define LWBTN_BUTTON_H

#include "Cpphand.h"
#include "lwbtn.h"

/**
 * @file        lwbtn_button.h
 * @brief       基于 LwBTN 的按键管理模块（应用层封装）
 *
 * 本模块对 LwBTN（轻量级按键库）进行了二次封装，为用户提供：
 *  - 更简单的按键初始化与处理接口
 *  - 每个按键独立的事件记录（单击、双击、长按）
 *  - 事件“读一次即清零”，避免重复触发
 *  - 与 HAL 库兼容的 GPIO 读取方式
 *
 * 使用方式：
 *  1. 调用 button_init() 初始化所有按键
 *  2. 在周期性任务中调用 button_process()（通常 1ms 或 10ms）
 *  3. 使用 button_get_event() 获取某个按键的事件
 *
 * 示例：
 *      ButtonEvent evt = button_get_event(1);
 *      if (evt == BTN_DOUBLE) {
 *          // 执行双击操作...
 *      }
 *
 *      LWBTN_CFG_TIME_KEEPALIVE_PERIOD   //长按时间间隔
 */


/* 按键事件类型 */
typedef enum {
    BTN_NONE = 0,    /**< 无事件 */
    BTN_SINGLE,      /**< 单击事件 */
    BTN_DOUBLE,      /**< 双击事件 */
    BTN_LONG         /**< 长按事件 */
} ButtonEvent;

/* 单个按键事件结构（每键独立） */
typedef struct {
    ButtonEvent event; /**< 当前事件（读取后清零） */
} ButtonInfo;

/* 导出函数 */
void button_init(void);
void button_process(void);

/* 获取某个按键事件（读一次就清零） */
ButtonEvent button_get_event(uint8_t key_id);

#endif //LWBTN_BUTTON_H
