#include "App_flight.h"

// 存储mpu6050的全部数据
Gyro_Acc_Data all_data = {0};
// 低通滤波上一次的值
Gyro_Data last_data = {0};
// 欧拉角
Euler_Data euler_angle = {0};
// 角速度累加
float gyro_z_sum = 0;
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
    // debug_printf(":%d,%d,%d\n", all_data.gyro.gyro_x, all_data.gyro.gyro_y, all_data.gyro.gyro_z);

    // 加速度波动比较大，使用卡尔曼滤波
    all_data.acc.acc_x = Common_Filter_KalmanFilter(&kfs[0], all_data.acc.acc_x);
    all_data.acc.acc_y = Common_Filter_KalmanFilter(&kfs[1], all_data.acc.acc_y);
    all_data.acc.acc_z = Common_Filter_KalmanFilter(&kfs[2], all_data.acc.acc_z);
    // 打印加速度
    // debug_printf(":%d,%d,%d\n", all_data.acc.acc_x, all_data.acc.acc_y, all_data.acc.acc_z);

    // // 通过加速度和角速度来计算飞机姿态，姿态解算
    // // 使用互补解算计算欧拉角，优先使用加速度解算，俯仰角和横滚角可以计算
    // euler_angle.pitch = atan2(all_data.acc.acc_x * 1.0, all_data.acc.acc_z) / 3.1415926 * 180;
    // euler_angle.roll = atan2(all_data.acc.acc_y * 1.0, all_data.acc.acc_z) / 3.1415926 * 180;

    // // 偏航角，只能使用角速度积分
    // // 16ADC转换为度每秒，量程是正负2000度每秒
    // gyro_z_sum += (all_data.gyro.gyro_z * 2000 / 32768) * 0.006;
    // euler_angle.yaw = gyro_z_sum;

    // 使用四元数解算姿态
    Common_IMU_GetEulerAngle(&all_data, &euler_angle, 0.006);

    // 打印欧拉角
    debug_printf(":%.2f,%.2f,%.2f\n", euler_angle.pitch, euler_angle.roll, euler_angle.yaw);
}
