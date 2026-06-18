#ifndef _APP_RECEIVE_DATA_H_
#define _APP_RECEIVE_DATA_H_

#include "Int_SI24R1.h"
#include "com_config.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
// 定义帧头校验3个字节
#define FRAME_HEAD_CHECK1 'w'
#define FRAME_HEAD_CHECK2 'r'
#define FRAME_HEAD_CHECK3 'j'

// 最大重试次数
#define MAX_RETRY_TIMES 100

typedef struct
{
    uint16_t thr;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    uint8_t shutdown;
    uint8_t fix_height;
} Remote_Data;

/**
 * @brief 接收遥控器发送的数据，解析结构体
 *
 * @return uint8_t  0: 解析数据成功，1: 解析数据失败
 */
uint8_t App_receive_data(void);

/**
 * @brief 判断遥控器是否连接
 *
 *
 */
// void App_remote_connect_state(uint8_t res);

/**
 * @brief 判断飞行状态 空闲 正常 定高 故障
 *
 *
 */

void App_process_flight_state(void);

/**
 * @brief 判断解锁状态
 *
 *
 */

static uint8_t App_process_unlock(void);

#endif
