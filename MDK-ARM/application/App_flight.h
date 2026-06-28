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
#include "Int_VL53L1X.h"

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

/**
 * @brief 定高pid处理
 *
 */
void App_flight_fix_height_PID_process(void);

/**
 * @brief 复位所有PID（清零积分和误差）
 * @note 在飞行状态切换（解锁/加锁/故障）时调用
 */
void App_flight_reset_all_pid(void);

/**
 * @brief 准备遥测数据（回传给遥控器显示）
 */
void App_flight_prepare_telemetry(Telemetry_Data *telem);
#endif
