#include "Cpphand.h"
#include "can.h"
#include <string.h>


//
//                            _ooOoo_
//                           o8888888o
//                           88" . "88
//                           (| -_- |)
//                            O\ = /O
//                        ____/`---'\____
//                      .   ' \\| |// `.
//                       / \\||| : |||// \
//                     / _||||| -:- |||||- \
//                       | | \\\ - /// | |
//                     | \_| ''\---/'' | |
//                      \ .-\__ `-` ___/-. /
//                   ___`. .' /--.--\ `. . __
//                ."" '< `.___\_<|>_/___.' >'"".
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                 \ \ `-. \_ __\ /__ _/ .-` / /
//         ======`-.____`-.___\_____/___.-`____.-'======
//                            `=---='
//
//         .............................................
//                  佛祖镇楼                  BUG辟易
//          佛曰:
//                  写字楼里写字间，写字间里程序员；
//                  程序人员写程序，又拿程序换酒钱。
//                  酒醒只在网上坐，酒醉还来网下眠；
//                  酒醉酒醒日复日，网上网下年复年。
//                  但愿老死电脑间，不愿鞠躬老板前；
//                  奔驰宝马贵者趣，公交自行程序员。
//                  别人笑我忒疯癫，我笑自己命太贱；
//                  不见满街漂亮妹，哪个归得程序员？



TimerHandle_t TimBlinkTimer;

SerialProtocol Serial(&huart1);

uint8_t u1_rx_buf[256];
uint8_t HexCharToInt(uint8_t c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}


void AllInit()
{
    Serial.Init(SerialProtocol::DMA);

    // 1. 配置过滤器（必须要！否则收不到任何数据）
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;                     // 过滤器组 0
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;// 32位宽
    sFilterConfig.FilterIdHigh = 0x0000;              // ID 全为 0
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;          // 掩码全为 0 (表示“我不挑食，所有ID都接收”)
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;// 收到的报文放进 FIFO0
    sFilterConfig.FilterActivation = ENABLE;          // 激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;          // CAN2 (如果只有CAN1，这行无所谓)

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        // 过滤器配置错误处理
    }

    // 2. 启动 CAN 模块 (你原来的代码)
    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        // 启动失败处理
    }


    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        // 开启中断失败
    }
}


void Key_Tick(void* argument)
{
    for (;;)
    {
        osDelay(2000);
    }
}


void Hardware_Tick(void* argument)
{
    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(5);
    }
}


void CAN_Send_Test(uint32_t id, uint8_t* data, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    // 限制最大长度为 8
    if (len > 8) len = 8;

    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 发送
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
}


void Message_Task(void* argument)
{
    for (;;)
    {
        osDelay(500);
    }
}
/**
  * @brief  CAN 接收回调函数 (FIFO0 有消息时自动触发)
  * @param  hcan: CAN 句柄
  */
/**
  * @brief  CAN 接收回调函数 (FIFO0 有消息时自动触发)
  * @param  hcan: CAN 句柄
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 格式化输出: "RX ID:123 L:8 D:01 02..."
            Serial.Printf("RX ID:0x%X L:%d D:", RxHeader.StdId, RxHeader.DLC);

            for (uint8_t i = 0; i < RxHeader.DLC; i++)
            {
                Serial.Printf("%02X ", RxData[i]);
            }
            Serial.Printf("\r\n");
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &huart1) // 判断对应串口
    {
        CAN_Send_Test(0x666, u1_rx_buf, Size);
        Serial.HandleRxData(Size);
        Serial.RestartRx();
    }
}


//定时器任务
void TimBlinkCallback(TimerHandle_t xTimer)
{
    button_process();
}


void StartBlinkTimer()
{
    TimBlinkTimer = xTimerCreate(
        "TimBlink",
        pdMS_TO_TICKS(50),
        pdTRUE,
        (void*)0,
        TimBlinkCallback
    );

    if (TimBlinkTimer != NULL)
    {
        xTimerStart(TimBlinkTimer, 0);
    }
}

