#include "Cpphand.h"



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


void AllInit()
{
    StartBlinkTimer();
    button_init();
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
    auto Motor = std::make_unique<TB67H450_Stepper>(
        &htim2, TIM_CHANNEL_4,
        &htim2, TIM_CHANNEL_3,
        IN_AP_GPIO_Port, IN_AP_Pin,
        IN_AM_GPIO_Port, IN_AM_Pin,
        IN_BP_GPIO_Port, IN_BP_Pin,
        IN_BM_GPIO_Port, IN_BM_Pin
    );

    Motor->Init(300);
    for (;;)
    {
        Motor->Step(1);
        osDelay(5);
    }
}


void Message_Task(void* argument)
{
    for (;;)
    {
        osDelay(50);
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
        pdMS_TO_TICKS(10),
        pdTRUE,
        (void*)0,
        TimBlinkCallback
    );

    if (TimBlinkTimer != NULL)
    {
        xTimerStart(TimBlinkTimer, 0);
    }
}

