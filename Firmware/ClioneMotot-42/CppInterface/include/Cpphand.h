#ifndef CPPHAND_H_
#define CPPHAND_H_

#ifdef __cplusplus
extern "C"{
#endif
/*------------------------C------------------------*/
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "spi.h"
#include "lwbtn_button.h"
    void AllInit();
    void C_To_Cpp(void);  //主程序函数
    void Key_Tick(void *argument);
    void Hardware_Tick(void *argument);
    void Message_Task(void *argument);
    void StartBlinkTimer();

#ifdef __cplusplus
}
/*-----------------------C++------------------------*/

#include "TB67H450_Stepper.h"
#include <memory>
#include <cstdarg>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <string>
#include <vector>

#endif
#endif