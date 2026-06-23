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

// 角速度结构体
typedef struct
{
    int16_t gyro_x; // 横滚角，roll，往右飞为正，往左飞为负
    int16_t gyro_y; // 俯仰角，pitch，往前飞为正，往后飞为负
    int16_t gyro_z; // 偏航角，yaw，逆时针为正，顺时针为负
} Gyro_Data;
// 加速度结构体
typedef struct
{
    int16_t acc_x; // 朝前加速度为正
    int16_t acc_y; // 朝左加速度为正
    int16_t acc_z; // 朝上加速度为正
} Acc_Data;
// 陀螺仪结构体
typedef struct
{
    Gyro_Data gyro;
    Acc_Data acc;
} Gyro_Acc_Data;
// 解算后的欧拉角结构体
typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_Data;

#endif /* __COM_CONFIG_H */
