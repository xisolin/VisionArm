#include "Cpphand.h"
#include "can.h"


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
// SerialShell Shell(&huart1);

SerialProtocol Serial(&huart1);
CanProtocol CanProt(&hcan, 0x123, true);


void AllInit()
{
    StartBlinkTimer();
    button_init();
    // Shell.Init();
    Serial.Init(SerialProtocol::DMA);

    if (!CanProt.Init()) {
        // 初始化失败处理，比如打印个错误
        Serial.Printf("CAN Init Failed!\r\n");
    } else {
        Serial.Printf("CAN Init Success!\r\n");
    }
}


void Key_Tick(void* argument)
{
    for (;;)
    {
        if (button_get_event(0) == BTN_SINGLE){

        }else if (button_get_event(0) == BTN_DOUBLE){

        }
        if (button_get_event(1) == BTN_SINGLE){

        }else if (button_get_event(1) == BTN_DOUBLE){

        }
        osDelay(20);
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


void Message_Task(void* argument)
{
    // 定义一个短字符串
    std::array<uint8_t,5> arr{0x01,0x02,0x03,0x04,0x05};

    for (;;)
    {
        CanProt.SendArrayWithCRC16(arr);
        osDelay(1000);
    //     std::vector<uint8_t> rxData;
    //     if (CanProt.Receive(rxData)) {
    //         // 收到完整包了！
    //         Serial.Printf("Recv CAN Frame! Len=%d\r\n", rxData.size());
    //         // 打印内容
    //         for(auto b : rxData) Serial.Printf("%02X ", b);
    //         Serial.Printf("\r\n");
    //     }
    }
}



/**
  * @brief  CAN 接收回调函数 (FIFO0 有消息时自动触发)
  * @param  hcan: CAN 句柄
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 判断是不是 CAN1
    if (hcan->Instance == CAN1)
    {
        CanProt.OnRxIrq();
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (huart == &huart1) // 判断对应串口
    {
        Serial.HandleRxData(Size);
        Serial.RestartRx();
    }
}


// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     // if (huart == Shell.GetHandle())
//     // {
//     //     Shell.HandleRxInterrupt();
//     // }
// }


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

