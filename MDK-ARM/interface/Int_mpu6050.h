#ifndef _INT_MPU6050_H_
#define _INT_MPU6050_H_

#include "i2c.h"
#include "com_config.h"
#include "freeRTOS.h"
#include "task.h"
#include "stdlib.h"
#include "stdio.h"

// mpu6050从机地址
#define MPU6050_ADDR 0x68
// mpu6050读写地址
#define MPU6050_ADDR_W 0xD0
#define MPU6050_ADDR_R 0xD1

// ============================================
// MPU6050 寄存器地址定义
// ============================================

// 电源管理寄存器
#define MPU6050_PWR_MGMT_1 0x6B // 电源管理寄存器1
#define MPU6050_PWR_MGMT_2 0x6C // 电源管理寄存器2

// 采样率和滤波器配置
#define MPU6050_SMPLRT_DIV 0x19 // 采样率分频器
#define MPU6050_CONFIG 0x1A     // 配置寄存器（低通滤波器）

// 量程配置
#define MPU6050_GYRO_CONFIG 0x1B  // 陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG 0x1C // 加速度计配置寄存器

// 数据寄存器（用于后续读取）
#define MPU6050_ACCEL_XOUT_H 0x3B // 加速度X轴高字节
#define MPU6050_ACCEL_XOUT_L 0x3C // 加速度X轴低字节
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41 // 温度高字节
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43 // 陀螺仪X轴高字节
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

// 设备ID
#define MPU6050_WHO_AM_I 0x75 // WHO_AM_I 寄存器（固定返回 0x68）

// MPU6050 中断配置寄存器
#define MPU6050_INT_ENABLE 0x38
// 用户配置寄存器
#define MPU6050_USER_CTRL 0x6A

/**
 * @brief 初始化mpu6050
 *
 *
 */
void Int_mpu6050_init(void);
/**
 * @brief 写入mpu6050寄存器
 *
 * @param reg 寄存器地址
 * @param data 数据
 */
void Int_mpu6050_write_reg(uint8_t reg, uint8_t data);
/**
 * @brief 读取mpu6050寄存器
 *
 * @param reg 寄存器地址
 * @param data 数据
 */
void Int_mpu6050_read_reg(uint8_t reg, uint8_t *data);

/**
 * @brief 获取角速度数据
 *
 * @param gyro 角速度数据
 */
void Int_mpu6050_get_Gyro(Gyro_Data *gyro);
/**
 * @brief 获取加速度数据
 *
 * @param accel 加速度数据
 */
void Int_mpu6050_get_Accel(Acc_Data *accel);
/**
 * @brief 获取所有数据
 *
 * @param data 所有数据
 */
void Int_mpu6050_GetAllData(Gyro_Acc_Data *data);

#endif
