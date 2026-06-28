#include "Int_mpu6050.h"

/* ========== 零偏校准值 ========== */
static int32_t gyro_x_offset;
static int32_t gyro_y_offset;
static int32_t gyro_z_offset;

static int32_t acc_x_offset;
static int32_t acc_y_offset;
static int32_t acc_z_offset;

/* ========== I2C 容错 ========== */
static Gyro_Acc_Data last_good_data = {0}; // 上一次成功读取的数据
static uint32_t      i2c_error_count = 0;   // I2C 错误计数

uint32_t Int_mpu6050_GetErrorCount(void)
{
    return i2c_error_count;
}

/* ========== 底层 I2C 读写（带返回值检查） ========== */

static HAL_StatusTypeDef Int_mpu6050_write_reg_safe(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_W, reg,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

static HAL_StatusTypeDef Int_mpu6050_read_reg_safe(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_R, reg,
                            I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

/**
 * @brief Burst read: 从 ACCEL_XOUT_H (0x3B) 开始连续读 14 字节
 *        6 轴 × 2 字节 + 温度 2 字节，利用 MPU6050 自动地址递增
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef Int_mpu6050_burst_read(uint8_t buf[14])
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_R, MPU6050_ACCEL_XOUT_H,
                            I2C_MEMADD_SIZE_8BIT, buf, 14, 100);
}

/**
 * @brief 从 burst read 的 14 字节缓冲区解析出陀螺仪 + 加速度数据
 */
static void Int_mpu6050_parse_burst(const uint8_t buf[14], Gyro_Acc_Data *data)
{
    // buf[0..5] : ACCEL_X/Y/Z (H,L)
    data->acc.acc_x = (int16_t)((buf[0] << 8) | buf[1]) - acc_x_offset;
    data->acc.acc_y = (int16_t)((buf[2] << 8) | buf[3]) - acc_y_offset;
    data->acc.acc_z = (int16_t)((buf[4] << 8) | buf[5]) - acc_z_offset;

    // buf[6..7] : TEMP (skip)

    // buf[8..13] : GYRO_X/Y/Z (H,L)
    data->gyro.gyro_x = (int16_t)((buf[8]  << 8) | buf[9])  - gyro_x_offset;
    data->gyro.gyro_y = (int16_t)((buf[10] << 8) | buf[11]) - gyro_y_offset;
    data->gyro.gyro_z = (int16_t)((buf[12] << 8) | buf[13]) - gyro_z_offset;
}

/* ========== 公开 API（保留旧接口兼容校准函数）========== */

void Int_mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    Int_mpu6050_write_reg_safe(reg, data);
}

void Int_mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    Int_mpu6050_read_reg_safe(reg, data);
}

/**
 * @brief 获取角速度（使用 burst read）
 */
void Int_mpu6050_get_Gyro(Gyro_Data *gyro)
{
    uint8_t buf[14];
    if (Int_mpu6050_burst_read(buf) == HAL_OK)
    {
        Gyro_Acc_Data tmp;
        Int_mpu6050_parse_burst(buf, &tmp);
        *gyro = tmp.gyro;
    }
    else
    {
        i2c_error_count++;
        // 用上次成功数据兜底
        *gyro = last_good_data.gyro;
    }
}

/**
 * @brief 获取加速度（使用 burst read）
 */
void Int_mpu6050_get_Accel(Acc_Data *accel)
{
    uint8_t buf[14];
    if (Int_mpu6050_burst_read(buf) == HAL_OK)
    {
        Gyro_Acc_Data tmp;
        Int_mpu6050_parse_burst(buf, &tmp);
        *accel = tmp.acc;
    }
    else
    {
        i2c_error_count++;
        *accel = last_good_data.acc;
    }
}

/**
 * @brief 获取所有数据 —— flight task 的主入口
 *        I2C 成功 → 解析数据，更新 last_good_data
 *        I2C 失败 → 丢弃，用上一帧数据兜底，避免飞控发散
 */
void Int_mpu6050_GetAllData(Gyro_Acc_Data *data)
{
    uint8_t buf[14];
    if (Int_mpu6050_burst_read(buf) == HAL_OK)
    {
        Int_mpu6050_parse_burst(buf, data);
        last_good_data = *data; // 记下最后一帧好数据
    }
    else
    {
        i2c_error_count++;
        *data = last_good_data; // 兜底：维持上一帧
    }
}

/* ========== 初始化和校准 ========== */

void Int_mpu6050_calibrate(void)
{
    // 1. 等待飞机停放平稳（连续 100 次加速度变化 < 400）
    Acc_Data current_acc = {0};
    Acc_Data last_acc = {0};
    uint8_t  count = 0;

    Int_mpu6050_get_Accel(&last_acc);
    while (count < 100)
    {
        Int_mpu6050_get_Accel(&current_acc);
        if (abs(current_acc.acc_x - last_acc.acc_x) < 400 &&
            abs(current_acc.acc_y - last_acc.acc_y) < 400 &&
            abs(current_acc.acc_z - last_acc.acc_z) < 400)
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

    // 2. 采集 100 组数据计算零偏均值
    Gyro_Acc_Data data = {0};
    int32_t gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
    int32_t acc_x_sum  = 0, acc_y_sum  = 0, acc_z_sum  = 0;

    for (uint8_t i = 0; i < 100; i++)
    {
        Int_mpu6050_GetAllData(&data);
        gyro_x_sum += data.gyro.gyro_x;
        gyro_y_sum += data.gyro.gyro_y;
        gyro_z_sum += data.gyro.gyro_z;
        acc_x_sum  += data.acc.acc_x;
        acc_y_sum  += data.acc.acc_y;
        acc_z_sum  += data.acc.acc_z - 16384; // Z 轴静止应为 1g = 16384 LSB
        vTaskDelay(6);
    }

    gyro_x_offset = gyro_x_sum / 100;
    gyro_y_offset = gyro_y_sum / 100;
    gyro_z_offset = gyro_z_sum / 100;
    acc_x_offset  = acc_x_sum  / 100;
    acc_y_offset  = acc_y_sum  / 100;
    acc_z_offset  = acc_z_sum  / 100;
}

void Int_mpu6050_init(void)
{
    uint8_t data;
    uint8_t retry;

    // 1. 复位芯片（带重试）
    for (retry = 0; retry < 5; retry++)
    {
        if (Int_mpu6050_write_reg_safe(MPU6050_PWR_MGMT_1, 0x80) == HAL_OK)
            break;
        vTaskDelay(10);
    }

    // 等待进入 sleep 模式
    vTaskDelay(50);
    data = 0;
    for (retry = 0; retry < 20; retry++)
    {
        if (Int_mpu6050_read_reg_safe(MPU6050_PWR_MGMT_1, &data) == HAL_OK
            && data == 0x40)
            break;
        vTaskDelay(5);
    }

    // 2. 退出 sleep，进入正常模式
    Int_mpu6050_write_reg_safe(MPU6050_PWR_MGMT_1, 0x00);
    vTaskDelay(10);

    // 3. 陀螺仪量程 ±2000dps
    Int_mpu6050_write_reg_safe(MPU6050_GYRO_CONFIG, 0x18);

    // 4. 加速度计量程 ±2g
    Int_mpu6050_write_reg_safe(MPU6050_ACCEL_CONFIG, 0x00);

    // 5. 关闭中断和 FIFO
    Int_mpu6050_write_reg_safe(MPU6050_INT_ENABLE, 0x00);
    Int_mpu6050_write_reg_safe(MPU6050_USER_CTRL,  0x00);

    // 6. 采样率 1kHz
    Int_mpu6050_write_reg_safe(MPU6050_SMPLRT_DIV, 0x01);

    // 7. 低通滤波器 188Hz
    Int_mpu6050_write_reg_safe(MPU6050_CONFIG, 1);

    // 8. 时钟源：PLL with X gyro reference
    Int_mpu6050_write_reg_safe(MPU6050_PWR_MGMT_1, 0x01);
    Int_mpu6050_write_reg_safe(MPU6050_PWR_MGMT_2, 0x00);
    vTaskDelay(10);

    // 9. 零偏校准
    Int_mpu6050_calibrate();
}
