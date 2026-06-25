#ifndef _APP_FLIGHT_H_
#define _APP_FLIGHT_H_

#include "Int_mpu6050.h"
#include "com_config.h"
#include "Com_debug.h"
#include "Com_filter.h"
#include "math.h"
#include "Com_imu.h"
#include "Com_pid.h"
#include "App_receive_data.h"
#include "Int_Motor.h"

/**
 * @brief 飞控任务初始化 MPU6050初始化    启动电机
 *
 */
void App_flight_init(void);

/**
 * @brief 获取欧拉角
 *
 */
void App_flight_get_euler_angle(void);

/**
 * @brief 根据欧拉角计算pid的目标值
 *
 */
void App_flight_pid_process(void);

/**
 * @brief 根据pid输出值控制电机
 */
void App_flight_control_motor(void);

#endif
