#include "App_freeRTOS_Task.h"

// LED结构体
LED_St left_top_led = {LED1_GPIO_Port, LED1_Pin};
LED_St right_top_led = {LED2_GPIO_Port, LED2_Pin};
LED_St right_bottom_led = {LED3_GPIO_Port, LED3_Pin};
LED_St left_bottom_led = {LED4_GPIO_Port, LED4_Pin};

// 当前遥控器连接状态
Remote_State remote_state = REMOTE_DISCONNECTED;

// 当前飞行状态
Flight_State flight_state = FLIGHT_IDLE;

// 遥控器数据
Remote_Data remote_data = {.thr = 0, .yaw = 500, .pitch = 500, .roll = 500, .shutdown = 0, .fix_height = 0};

// 电源管理任务
//  使用宏定义代替电源任务栈大小，优先级和任务句柄
void Power_Task(void *pvParameters);
#define POWER_TASK_STACK_SIZE 128
// 任务优先级，数字越大优先级越高，4是最高优先级，0是最低优先级
#define POWER_TASK_PRIORITY 4
// 定义任务周期10s
#define POWER_TASK_PERIOD 10000
TaskHandle_t powerTaskHandle = NULL;

// flight电机任务
void Flight_Motor_Task(void *pvParameters);
#define FLIGHT_MOTOR_TASK_STACK_SIZE 128
#define FLIGHT_MOTOR_TASK_PRIORITY 3
#define FLIGHT_MOTOR_TASK_PERIOD 6
TaskHandle_t flightMotorTaskHandle = NULL;

// LED状态任务
void LED_Status_Task(void *pvParameters);
#define LED_STATUS_TASK_STACK_SIZE 128
#define LED_STATUS_TASK_PRIORITY 1
#define LED_STATUS_TASK_PERIOD 100
TaskHandle_t ledStatusTaskHandle = NULL;

// 通信任务
void Com_Task(void *pvParameters);
#define COM_TASK_STACK_SIZE 128
#define COM_TASK_PRIORITY 2
#define COM_TASK_PERIOD 6
TaskHandle_t comTaskHandle = NULL;

void App_freeRTOS_Start(void)
{
    // 1. 创建电源启动任务
    xTaskCreate(Power_Task, "Power Task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &powerTaskHandle);

    // 2. 创建飞行电机任务
    xTaskCreate(Flight_Motor_Task, "Flight Motor Task", FLIGHT_MOTOR_TASK_STACK_SIZE, NULL, FLIGHT_MOTOR_TASK_PRIORITY, &flightMotorTaskHandle);

    // 3. 创建LED状态任务
    xTaskCreate(LED_Status_Task, "LED Status Task", LED_STATUS_TASK_STACK_SIZE, NULL, LED_STATUS_TASK_PRIORITY, &ledStatusTaskHandle);

    // 4. 创建通信任务
    xTaskCreate(Com_Task, "Com Task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &comTaskHandle);

    // 5. 启动调度器
    vTaskStartScheduler();
}

// 创建电源管理任务
void Power_Task(void *pvParameters)
{
    // 获取当前系统时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 每十秒执行一次，使用vtaskdelayuntil更精确地控制时间
        // vTaskDelayUntil(&xLastWakeTime, POWER_TASK_PERIOD);

        // // 启动电源
        // Int_IP5305T_Start();

        // 使用freertos直接任务通知接收方法实现10s处理一次，收到通知res为1，超时res为0
        uint32_t res = ulTaskNotifyTake(pdTRUE, POWER_TASK_PERIOD);
        if (res != 0)
        {
            // 收到关机通知
            Int_IP5305T_Shutdown();
        }
        else
        {
            // 如果没有收到通知，则启动电源
            Int_IP5305T_Start();
        }
    }
}

// 创建飞行电机任务
void Flight_Motor_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // 初始化mpu6050
    App_flight_init();
    while (1)
    {
        // 1.根据mpu6050测量的数据，姿态解算得到欧拉角
        App_flight_get_euler_angle();
        // 2.得到欧拉角后，进行pid计算
        App_flight_pid_process();
        // 3.根据pid计算的结果，控制电机转速
        App_flight_control_motor();

        // 每秒执行一次，使用vtaskdelayuntil更精确地控制时间
        vTaskDelayUntil(&xLastWakeTime, FLIGHT_MOTOR_TASK_PERIOD);
    }
}

// 创建LED状态任务
void LED_Status_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t counter = 0; // 任务100ms执行一次，counter每次加1，counter%5==0时执行一次待机状态闪烁，counter%2==0时执行一次飞行状态闪烁
    while (1)
    {
        counter++; // 每次任务执行后counter加1
        // LED状态
        // 1.前两个灯表示遥控器连接状态，遥控器连接时前面两个LED灯常亮，未连接时LED灯熄灭
        if (remote_state == REMOTE_CONNECTED)
        {
            // 启动前面两个LED灯
            Int_LED_On(&left_top_led);
            Int_LED_On(&right_top_led);
        }
        else if (remote_state == REMOTE_DISCONNECTED)
        {
            // 熄灭前面两个LED灯
            Int_LED_Off(&left_top_led);
            Int_LED_Off(&right_top_led);
        }

        // 2.后两个灯表示飞行状态，待机状态时后面两个LED灯缓慢闪烁，正常飞行状态时后面两个LED灯快速闪烁，定高状态时后面两个LED灯常亮，故障状态时后面两个LED灯熄灭
        switch (flight_state)
        {
        case FLIGHT_IDLE:
            // 待机状态，后面两个LED灯缓慢闪烁，500ms翻转一次LED状态
            if (counter % 5 == 0) // 每500ms执行一次
            {
                Int_LED_Toggle(&right_bottom_led);
                Int_LED_Toggle(&left_bottom_led);
            }
            break;
        case FLIGHT_NORMAL:
            // 正常飞行状态，后面两个LED灯快速闪烁，200ms翻转一次LED状态
            if (counter % 2 == 0) // 每200ms执行一次
            {
                Int_LED_Toggle(&right_bottom_led);
                Int_LED_Toggle(&left_bottom_led);
            }
            break;
        case FLIGHT_FIX_HEIGHT:
            // 定高状态，后面两个LED灯常亮
            Int_LED_On(&right_bottom_led);
            Int_LED_On(&left_bottom_led);
            break;
        case FLIGHT_FAIL:
            // 故障状态，后面两个LED灯熄灭
            Int_LED_Off(&right_bottom_led);
            Int_LED_Off(&left_bottom_led);
            break;
        }
        if (counter == 10) // 防止counter溢出，每10次重置一次counter
        {
            counter = 0;
        }

        // 每秒执行一次，使用vtaskdelayuntil更精确地控制时间
        vTaskDelayUntil(&xLastWakeTime, LED_STATUS_TASK_PERIOD);
    }
}
uint8_t com_data[TX_PLOAD_WIDTH] = {0};
// 通信任务
void Com_Task(void *pvParameters)
{
    // 获取当前系统时间
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // 调用si24r1接收数据，处理接收到的数据
        App_receive_data();

        // 处理关机命令
        if (remote_data.shutdown == 1)
        {
            // //方式1直接调用关机函数
            // Int_IP5305T_Shutdown();

            // 方式2，使用freertos直接任务通知，通知电源任务，执行关机
            xTaskNotifyGive(powerTaskHandle);
        }
        // 处理飞行状态
        App_process_flight_state();

        // 每十秒执行一次，使用vtaskdelayuntil更精确地控制时间
        vTaskDelayUntil(&xLastWakeTime, COM_TASK_PERIOD);
    }
}
