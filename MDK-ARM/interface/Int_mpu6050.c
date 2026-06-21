#include "Int_mpu6050.h"

void Int_mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_W, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}
void Int_mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_R, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

int32_t gyro_x_offset;
int32_t gyro_y_offset;
int32_t gyro_z_offset;

int32_t acc_x_offset;
int32_t acc_y_offset;
int32_t acc_z_offset;

/**
 * @brief 零偏校准mpu6050
 */
void Int_mpu6050_calibrate(void)
{
    // 1.判断飞机是否停放平稳
    // 平放是否停稳标志，前后两次加速度差值小于400
    Acc_Data current_acc = {0};
    Acc_Data last_acc = {0};
    uint8_t count = 0;
    Int_mpu6050_get_Accel(&last_acc);
    while (count < 100)
    {
        Int_mpu6050_get_Accel(&current_acc);
        if (abs(current_acc.acc_x - last_acc.acc_x) < 400 && abs(current_acc.acc_y - last_acc.acc_y) < 400 && abs(current_acc.acc_z - last_acc.acc_z) < 400)
        {
            count++;
        }
        else
        {
            count = 0;
        }
        last_acc = current_acc;
        vTaskDelay(6);
    }
    // 2.飞机平稳后，进行零偏校准
    Gyro_Acc_Data data = {0};
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;

    int32_t acc_x_sum = 0;
    int32_t acc_y_sum = 0;
    int32_t acc_z_sum = 0;

    for (uint8_t i = 0; i < 100; i++)
    {
        Int_mpu6050_GetAllData(&data);
        // 重新读取角速度
        gyro_x_sum += (data.gyro.gyro_x - 0);
        gyro_y_sum += (data.gyro.gyro_y - 0);
        gyro_z_sum += (data.gyro.gyro_z - 0);

        acc_x_sum += (current_acc.acc_x - 0);
        acc_y_sum += (current_acc.acc_y - 0);
        // z轴初始化加速度为1g
        acc_z_sum += (current_acc.acc_z - 16384);
        // 添加延迟多次测量
        vTaskDelay(6);
    }
    gyro_x_offset = gyro_x_sum / 100;
    gyro_y_offset = gyro_y_sum / 100;
    gyro_z_offset = gyro_z_sum / 100;

    acc_x_offset = acc_x_sum / 100;
    acc_y_offset = acc_y_sum / 100;
    acc_z_offset = acc_z_sum / 100;
}

/**
 * @brief 初始化mpu6050
 */
void Int_mpu6050_init(void)
{
    // 1.重启芯片，重置所有寄存器，设置陀螺仪采样率为1000Hz，设置加速度计采样率为1000Hz，设置陀螺仪量程为2000dps，设置加速度计量程为8g
    Int_mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x80);
    uint8_t data = 0;
    // MPU6050_PWR_MGMT_1值为0x40，是在sleep模式
    while (data != 0x40)
    {
        Int_mpu6050_read_reg(MPU6050_PWR_MGMT_1, &data);
    }
    // 写入0x00，进入正常模式
    Int_mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    // 选择合适量程，越小精度越高
    // 陀螺仪配置寄存器,2000度
    Int_mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x18);
    // 加速度计配置寄存器,+_2g
    Int_mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x00);
    // 关闭使能中断，用不到
    Int_mpu6050_write_reg(MPU6050_INT_ENABLE, 0x00);
    // 用户配置寄存器，用不到fifo队列
    Int_mpu6050_write_reg(MPU6050_USER_CTRL, 0x00);
    // 设置采样率为1000Hz
    Int_mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x01);
    // 配置低通滤波器，188Hz
    Int_mpu6050_write_reg(MPU6050_CONFIG, 1);
    // 配置使用的系统时钟为添加pll，1000Hz
    Int_mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);
    // 使用加速度传感器和角速度传感器
    Int_mpu6050_write_reg(MPU6050_PWR_MGMT_2, 0x00);
    // 进行零偏校准
    Int_mpu6050_calibrate();
}

/**
 * @brief 获取角速度数据
 *
 * @param gyro 角速度数据
 */
void Int_mpu6050_get_Gyro(Gyro_Data *gyro)
{
    // 角速度寄存器地址从0x43开始，8高位在前 低8位在后， xyz顺序
    uint8_t height = 0;
    uint8_t low = 0;
    Int_mpu6050_read_reg(MPU6050_GYRO_XOUT_H, &height);
    Int_mpu6050_read_reg(MPU6050_GYRO_XOUT_L, &low);
    gyro->gyro_x = (int16_t)((height << 8) | low) - gyro_x_offset;
    Int_mpu6050_read_reg(MPU6050_GYRO_YOUT_H, &height);
    Int_mpu6050_read_reg(MPU6050_GYRO_YOUT_L, &low);
    gyro->gyro_y = (int16_t)((height << 8) | low) - gyro_y_offset;
    Int_mpu6050_read_reg(MPU6050_GYRO_ZOUT_H, &height);
    Int_mpu6050_read_reg(MPU6050_GYRO_ZOUT_L, &low);
    gyro->gyro_z = (int16_t)((height << 8) | low) - gyro_z_offset;
}
/**
 * @brief 获取加速度数据
 *
 * @param accel 加速度数据
 */
void Int_mpu6050_get_Accel(Acc_Data *accel)
{
    uint8_t height = 0;
    uint8_t low = 0;
    Int_mpu6050_read_reg(MPU6050_ACCEL_XOUT_H, &height);
    Int_mpu6050_read_reg(MPU6050_ACCEL_XOUT_L, &low);
    accel->acc_x = (int16_t)((height << 8) | low) - acc_x_offset;
    Int_mpu6050_read_reg(MPU6050_ACCEL_YOUT_H, &height);
    Int_mpu6050_read_reg(MPU6050_ACCEL_YOUT_L, &low);
    accel->acc_y = (int16_t)((height << 8) | low) - acc_y_offset;
    Int_mpu6050_read_reg(MPU6050_ACCEL_ZOUT_H, &height);
    Int_mpu6050_read_reg(MPU6050_ACCEL_ZOUT_L, &low);
    accel->acc_z = (int16_t)((height << 8) | low) - acc_z_offset;
}
/**
 * @brief 获取所有数据
 *
 * @param data 所有数据
 */
void Int_mpu6050_GetAllData(Gyro_Acc_Data *data)
{
    // 获取六个数据
    Int_mpu6050_get_Gyro(&data->gyro);
    Int_mpu6050_get_Accel(&data->acc);
}
