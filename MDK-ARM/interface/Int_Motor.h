#ifndef INT_MOTOR_H
#define INT_MOTOR_H

#include "tim.h"
#include "Com_debug.h"

// 使用结构体封装电机数据
typedef struct
{
    TIM_HandleTypeDef *tim; // 定时器句柄
    uint16_t channel;       // 定时器通道
    int16_t speed;          // 电机速度，范围0-1000，对应定时器的周期

} Motor_St;

/**
 * 启动电机
 * @param motor 电机结构体指针
 */
void Int_Motor_Start(Motor_St *motor);

/**
 * 设置电机速度,传入的是比较值，最大1000，默认200
 * @param motor 电机结构体指针
 */
void Int_Motor_SetSpeed(Motor_St *motor);

#endif // INT_MOTOR_H
