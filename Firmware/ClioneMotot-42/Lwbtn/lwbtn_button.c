#include "lwbtn_button.h"

#define BUTTON_COUNT 2

/* ========= 全局按键事件表（每键一个） ========= */
static ButtonInfo g_btnInfo[BUTTON_COUNT];

/* ========= 实例化 GPIO 参数 ========= */
static lwbtn_argdata_port_pin_state_t key_gpios[BUTTON_COUNT] = {
    { .port = KEY1_GPIO_Port, .pin = (void *)KEY1_Pin, .state = 0 },
    { .port = KEY2_GPIO_Port, .pin = (void *)KEY2_Pin, .state = 0 },
};

/* ========= 创建 LwBTN 按钮对象 ========= */
static lwbtn_btn_t buttons[BUTTON_COUNT];

/* ========= GPIO 读取函数（提供给 lwbtn） ========= */
static uint8_t button_read_state(lwbtn_t* lw, lwbtn_btn_t* btn)
{
    lwbtn_argdata_port_pin_state_t* cfg = btn->arg;
    uint8_t val = HAL_GPIO_ReadPin(cfg->port, cfg->pin);
    return (val == cfg->state);
}

/* ========= LwBTN 回调事件处理 ========= */
static void button_event_handler(lwbtn_t* lw, lwbtn_btn_t* btn, lwbtn_evt_t evt)
{
    uint8_t id = (uint8_t)(btn - lw->btns); // 按键编号 0 ~ N-1

    switch (evt)
    {
    case LWBTN_EVT_ONCLICK: // 单击/双击
        if (btn->click.cnt == 2)
        {
            g_btnInfo[id].event = BTN_DOUBLE;
            btn->click.cnt = 0;
        }
        else
        {
            g_btnInfo[id].event = BTN_SINGLE;
        }
        break;

    case LWBTN_EVT_KEEPALIVE: // 长按事件
        g_btnInfo[id].event = BTN_LONG;
        break;

    default:
        break;
    }
}

/* ========== 初始化按键模块 ========== */
/**
 * @brief 初始化按键模块
 *
 * - 绑定 GPIO 配置
 * - 初始化 LwBTN 按键对象
 * - 清空所有事件状态
 *
 * 必须在使用按键功能前调用一次。
 */
void button_init(void)
{
    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        buttons[i].arg = &key_gpios[i];
        g_btnInfo[i].event = BTN_NONE;
    }

    lwbtn_init_ex(NULL,
                  buttons,
                  BUTTON_COUNT,
                  button_read_state,
                  button_event_handler);
}

/* ========== 周期性调用（例如在1ms任务中） ========== */
/**
 * @brief 按键处理函数
 *
 * 必须周期性调用（建议每 1ms 或 10ms 调用一次）。
 * 用于：
 *  - 读取 GPIO
 *  - 更新按键状态
 *  - 触发 LwBTN 的事件逻辑
 */
void button_process(void)
{
    lwbtn_process(HAL_GetTick());
}

/* ========== 用户读取某键事件（读一次即清零） ========== */

/**
 * @brief 获取某个按键的事件（读一次即清零）
 *
 * @param key_id 按键编号（0 ~ N-1）
 * @return ButtonEvent 返回当前事件，没有事件则返回 BTN_NONE
 *
 * 使用说明：
 *  - 每次读取后，该按键事件会被自动清除
 *  - 避免重复触发同一事件
 */
ButtonEvent button_get_event(uint8_t key_id)
{
    if (key_id >= BUTTON_COUNT)
        return BTN_NONE;

    ButtonEvent evt = g_btnInfo[key_id].event;
    g_btnInfo[key_id].event = BTN_NONE;
    return evt;
}
