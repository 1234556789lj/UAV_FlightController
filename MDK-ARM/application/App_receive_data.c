#include "App_receive_data.h"
#include "App_flight.h"

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

// 按下定高后的高度
uint16_t fix_height = 0;

// 通信状态追踪
static uint32_t last_valid_packet_tick = 0; // 最后一次收到有效数据包的时间
static uint8_t  failsafe_active = 0;         // 失控保护激活标志

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

    // 1. 接收数据
    uint8_t status = Int_SI24R1_RxPacket(rx_buff);

    // 2. 如果收到原始数据，进行帧校验和解析
    if (status == 0)
    {
        // 帧头校验
        if (rx_buff[0] != FRAME_HEAD_CHECK1 || rx_buff[1] != FRAME_HEAD_CHECK2 || rx_buff[2] != FRAME_HEAD_CHECK3)
        {
            status = 1; // 帧头错误，视为无效包
        }
        else
        {
            // 帧尾校验
            uint32_t sum = 0;
            for (uint8_t i = 0; i < 13; i++)
            {
                sum += rx_buff[i];
            }
            uint32_t reci_sum = (rx_buff[13] << 24) | (rx_buff[14] << 16) | (rx_buff[15] << 8) | rx_buff[16];
            if (sum != reci_sum)
            {
                status = 1; // 校验失败
            }
        }
    }

    // 3. 更新连接状态（基于tick的超时机制，替代固定次数重试）
    uint32_t now_tick = xTaskGetTickCount();

    if (status == 0)
    {
        // 收到有效数据包 → 解析并保存
        remote_data.thr      = (rx_buff[3] << 8) | rx_buff[4];
        remote_data.yaw      = (rx_buff[5] << 8) | rx_buff[6];
        remote_data.pitch    = (rx_buff[7] << 8) | rx_buff[8];
        remote_data.roll     = (rx_buff[9] << 8) | rx_buff[10];
        remote_data.shutdown = rx_buff[11];
        remote_data.fix_height = rx_buff[12];

        // 更新最后收到有效包的时间戳
        last_valid_packet_tick = now_tick;

        // 立即恢复连接状态
        remote_state = REMOTE_CONNECTED;
        failsafe_active = 0;

        // debug_printf(":%d,%d,%d,%d,%d,%d\n", remote_data.thr, remote_data.yaw, remote_data.pitch, remote_data.roll, remote_data.shutdown, remote_data.fix_height);
        return 0;
    }

    // 4. 未收到有效数据 → 检查超时
    uint32_t elapsed = now_tick - last_valid_packet_tick;

    // LED指示用短超时（快速反映信号状态）
    if (elapsed >= REMOTE_LOST_TICK_MS)
    {
        remote_state = REMOTE_DISCONNECTED;
    }

    // 失控保护用长超时（避免短暂干扰触发炸机）
    if (elapsed >= FAILSAFE_TICK_MS)
    {
        failsafe_active = 1;
    }

    return 1;
}

// 处理飞行状态
void App_process_flight_state(void)
{
    switch (flight_state)
    {
    case FLIGHT_IDLE:
        if (App_process_unlock() == 0)
        {
            flight_state = FLIGHT_NORMAL;
            // 解锁成功，清零所有PID积分
            App_flight_reset_all_pid();
            thr_state = FREE;
        }
        break;

    case FLIGHT_NORMAL:
        // 判断进入定高
        if (remote_data.fix_height == 1)
        {
            flight_state = FLIGHT_FIX_HEIGHT;
            remote_data.fix_height = 0;
            fix_height = Int_VL53L1X_GetDistance();
        }
        // 失控保护：持续断连超过FAILSAFE_TICK_MS才触发故障
        if (failsafe_active)
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
        // 失控保护
        if (failsafe_active)
        {
            flight_state = FLIGHT_FAIL;
        }
        break;

    case FLIGHT_FAIL:
        // 故障状态：由电机控制函数负责缓慢降落
        // 当油门降到0后，自动切回空闲
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
