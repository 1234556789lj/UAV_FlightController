#include "App_receive_data.h"

uint8_t rx_buff[TX_PLOAD_WIDTH] = {0};

// 遥控器连接状态
extern Remote_State remote_state;
// 飞行状态
extern Flight_State flight_state;
// 遥控器数据
extern Remote_Data remote_data;
// 油门状态
Thr_State thr_state = FREE;
// 记录最大max油门时间
uint32_t enter_max_time = 0;
// 记录最小min油门时间
uint32_t enter_min_time = 0;

uint8_t retry_count = 0;
/**
 * @brief 解锁函数,空闲解锁到正常状态，0: 解锁成功，1: 解锁失败
 *
 */

static uint8_t App_process_unlock(void)
{
    switch (thr_state)
    {
        // 空闲状态，等待解锁
    case FREE:
        if (remote_data.thr > 650)
        {
            thr_state = MAX;
            // 记录进入最大油门时间，freertos带的系统计数,ms
            enter_max_time = xTaskGetTickCount();
        }
        break;
    case MAX:
        if (remote_data.thr < 650)
        {
            // 油门低于最大值max就开始进入leave max状态或退回free
            if (xTaskGetTickCount() - enter_max_time >= 1000)
            {
                thr_state = LEAVE_MAX;
            }
            else
            {
                thr_state = FREE;
            }
        }
        break;
    case LEAVE_MAX:
        if (remote_data.thr <= 100)
        {
            // 油门小于100，进入min
            thr_state = MIN;
            enter_min_time = xTaskGetTickCount();
        }
        break;
    case MIN:
        // 进入min时间小于1s，退回free
        if (xTaskGetTickCount() - enter_min_time < 1000)
        {
            if (remote_data.thr > 100)
            {
                thr_state = FREE;
            }
        }
        else
        {
            // 进入min时间大于1s，解锁成功
            thr_state = UNLOCK;
        }
        break;
    case UNLOCK:
        return 0; // 解锁成功
    default:
        return 1;
    }
    return 1; // 未解锁
}

/**
 * @brief 接收遥控器发送的数据，解析结构体
 *
 * @return uint8_t  0: 解析数据成功，1: 解析数据失败
 */
uint8_t App_receive_data(void)
{
    // 每次接收数据之前先清空rx_buff
    memset(rx_buff, 0, TX_PLOAD_WIDTH);
    // 1. 接收数据，收到的数据保存在rx_buff中
    uint8_t status = Int_SI24R1_RxPacket(rx_buff);
    // 判断遥控器连接状态，收到数据遥控器连接状态置为连接成功，否则置为连接失败
    if (status == 0)
    {
        // 接收数据成功一次 即为连接成功
        // 此处使用的全局变量 只有当前一个地方会修改 LED灯控任务当中是读取使用
        remote_state = REMOTE_CONNECTED;
        retry_count = 0;
    }
    else
    {
        // 接收数据失败 即重试
        retry_count++;
        if (retry_count >= MAX_RETRY_TIMES)
        {
            remote_state = REMOTE_DISCONNECTED;
            retry_count = 0;
        }
    }
    // 2. 如果没收到数据，直接返回失败
    if (status != 0)
    {
        return 1;
    }
    // // 接收数据
    // Int_SI24R1_RxPacket(rx_buff);
    // if (strlen((char *)rx_buff) == 0)
    // {
    //     return 1;
    // }

    // 1.帧头校验
    if (rx_buff[0] != FRAME_HEAD_CHECK1 || rx_buff[1] != FRAME_HEAD_CHECK2 || rx_buff[2] != FRAME_HEAD_CHECK3)
    {
        return 1;
    }

    // 2.帧尾校验
    uint32_t sum;
    for (uint8_t i = 0; i < 13; i++)
    {
        sum += rx_buff[i];
    }

    // 高位先行
    uint32_t reci_sum = (rx_buff[13] << 24) | (rx_buff[14] << 16) | (rx_buff[15] << 8) | rx_buff[16];
    // 计算收到的13位数据的和，与后四位的和比较，如果相等，则校验通过
    if (sum != reci_sum)
    {
        return 1;
    }

    // 3.保存并解析数据
    remote_data.thr = (rx_buff[3] << 8) | rx_buff[4];
    remote_data.yaw = (rx_buff[5] << 8) | rx_buff[6];
    remote_data.pitch = (rx_buff[7] << 8) | rx_buff[8];
    remote_data.roll = (rx_buff[9] << 8) | rx_buff[10];
    remote_data.shutdown = rx_buff[11];
    remote_data.fix_height = rx_buff[12];

    // 串口打印
    // debug_printf(":%d,%d,%d,%d,%d,%d\n", remote_data.thr, remote_data.yaw, remote_data.pitch, remote_data.roll, remote_data.shutdown, remote_data.fix_height);

    return 0;
}

// 处理飞行状态
void App_process_flight_state(void)
{
    // 使用状态机逻辑实现

    switch (flight_state)
    {
    case FLIGHT_IDLE:
        if (App_process_unlock() == 0)
        {
            flight_state = FLIGHT_NORMAL;
            // 每一次解锁成功，清空解锁状态，油门
            thr_state = FREE;
        }
        break;
    case FLIGHT_NORMAL:
        // 判断进入定高
        if (remote_data.fix_height == 1)
        {
            flight_state = FLIGHT_FIX_HEIGHT;
            remote_data.fix_height = 0;
        }
        // 判断失联进入故障
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FLIGHT_FAIL;
        }

        break;
    case FLIGHT_FIX_HEIGHT:
        // 取消定高
        if (remote_data.fix_height == 1)
        {
            flight_state = FLIGHT_NORMAL;
            remote_data.fix_height = 0;
        }
        // 判断失联进入故障
        if (remote_state == REMOTE_DISCONNECTED)
        {
            flight_state = FLIGHT_FAIL;
        }

        break;
    case FLIGHT_FAIL:
        // 处理失联故障，缓慢停下电机，然后进入空闲状态
        vTaskDelay(1);
        flight_state = FLIGHT_IDLE;
        break;
    }
}

// void App_remote_connect_state(uint8_t res)
// {
//     if (res == 0)
//     {
//         // 接收数据成功一次 即为连接成功
//         // 此处使用的全局变量 只有当前一个地方会修改 LED灯控任务当中是读取使用
//         remote_state = REMOTE_CONNECTED;
//         retry_count = 0;
//     }
//     else if (res == 1)
//     {
//         // 接收数据失败 即重试
//         retry_count++;
//         if (retry_count >= MAX_RETRY_TIMES)
//         {
//             remote_state = REMOTE_DISCONNECTED;
//             retry_count = 0;
//         }
//     }
// }
