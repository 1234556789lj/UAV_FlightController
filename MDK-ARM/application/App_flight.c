#include "App_flight.h"

// 存储mpu6050的全部数据
Gyro_Acc_Data all_data = {0};
// 低通滤波上一次的值
Gyro_Data last_data = {0};
/**
 * @brief 获取欧拉角
 *
 */
void App_flight_get_euler_angle(void)
{
    // 获取mpu6050的全部数据
    Int_mpu6050_GetAllData(&all_data);

    // 对角速度进行低通滤波
    all_data.gyro.gyro_x = Common_Filter_LowPass(all_data.gyro.gyro_x, last_data.gyro_x);
    all_data.gyro.gyro_y = Common_Filter_LowPass(all_data.gyro.gyro_y, last_data.gyro_y);
    all_data.gyro.gyro_z = Common_Filter_LowPass(all_data.gyro.gyro_z, last_data.gyro_z);
    // 更新上一次的值
    last_data = all_data.gyro;

    // 打印角速度到串口
    debug_printf(":%d,%d,%d\n", all_data.gyro.gyro_x, all_data.gyro.gyro_y, all_data.gyro.gyro_z);
    // 打印加速度
    // debug_printf(":%d,%d,%d\n", all_data.acc.acc_x, all_data.acc.acc_y, all_data.acc.acc_z);
}
