#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include "main.h"

// 枚举遥控器连接状态
typedef enum
{
    REMOTE_CONNECTED = 0,   // 已连接
    REMOTE_DISCONNECTED = 1 // 未连接
} Remote_State;

// 油门解锁状态机
typedef enum
{
    FREE = 0,
    MIN,
    MAX,
    LEAVE_MAX,
    UNLOCK
} Thr_State;

// 飞行状态
typedef enum
{
    FLIGHT_IDLE = 0,       // 待机状态
    FLIGHT_NORMAL = 1,     // 正常飞行状态
    FLIGHT_FIX_HEIGHT = 2, // 定高状态
    FLIGHT_FAIL = 3        // 故障状态
} Flight_State;

#endif /* __COM_CONFIG_H */
