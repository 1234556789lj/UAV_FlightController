#ifndef _APP_FLIGHT_H_
#define _APP_FLIGHT_H_

#include "Int_mpu6050.h"
#include "com_config.h"
#include "Com_debug.h"
#include "Com_filter.h"
#include "math.h"
#include "Com_imu.h"

/**
 * @brief 获取欧拉角
 *
 */
void App_flight_get_euler_angle(void);

#endif
