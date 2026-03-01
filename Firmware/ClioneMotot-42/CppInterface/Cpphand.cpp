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



ADC_DRIVER  ADC_Driver(&hadc1);
TimerHandle_t TimBlinkTimer;
SerialShell Shell(&huart1);
Thermistor_NTC NTC(3300,0.0,10);

CanProtocol CanProt(&hcan, 0x123, true);
MT6816 Encoder(&hspi1, MT6816_CS_GPIO_Port, MT6816_CS_Pin);

TB67H450_Stepper myMotor(&htim2, TIM_CHANNEL_4, &htim2, TIM_CHANNEL_3);

void AllInit()
{
    StartBlinkTimer();
    button_init();
    Shell.Init();
    myMotor.Init();

    HAL_TIM_Base_Start_IT(&htim3);
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

    Encoder.Init();
    ADC_Driver.Init();
    for (;;)
    {
       // volatile float current_angle = Encoder.GetAngleDegrees();
       volatile auto adc2 = NTC.Update(ADC_Driver.getRaw(1)) ;

        osDelay(5);
    }
}
  

extern "C" void TIM1_20KHz() {

}

extern "C" void TIM3_1KHz() {
    static uint8_t carrot_angle = 0;
    static uint16_t speed_divider = 0;

    myMotor.SetFocCurrentVector(carrot_angle, 500);

    speed_divider++;
    if (speed_divider >= 20) {
        speed_divider = 0;
        carrot_angle++;
    }
}



void Message_Task(void* argument)
{
    // 定义一个短字符串
    std::array<uint8_t,5> arr{0x01,0x02,0x03,0x04,0x05};

    for (;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
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



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == Shell.GetHandle())
    {
        Shell.HandleRxInterrupt();
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

