#ifndef APP_FREERTOS_TASK_H
#define APP_FREERTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "com_config.h"
#include "Com_debug.h"
#include "Int_IP5305T.h"
#include "Int_Motor.h"
#include "Int_LED.h"
#include "Int_SI24R1.h"
#include "App_receive_data.h"
#include "App_flight.h"
#include "Int_mpu6050.h"

// 启动freeRTOS操作系统
void App_freeRTOS_Start(void);

#endif // APP_FREERTOS_TASK_H
