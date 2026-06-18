#include "Int_Motor.h"

// 启动电机
void Int_Motor_Start(Motor_St *motor)
{
    HAL_TIM_PWM_Start(motor->tim, motor->channel);
}

// 四个电机就不用单独写四个函数了，直接传入结构体指针就行了
void Int_Motor_SetSpeed(Motor_St *motor)
{
    if (motor->speed > 1000)
    {
        motor->speed = 1000; // 限制速度最大值为1000
    }
    else
    {
        __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, motor->speed);
        }
}
