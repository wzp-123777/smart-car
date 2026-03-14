/**
 * @file    mpu6050.h
 * @brief   MPU6050 陀螺仪/加速度计模块（I2C通信）
 * @note    用于辅助转弯纠偏，获取偏航角(Yaw)
 *          使用硬件I2C1 (PB8=SCL, PB9=SDA)
 */
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h"

/* ==================== MPU6050 寄存器定义 ==================== */
#define MPU6050_ADDR            0xD0    /* I2C地址（AD0接地=0x68，左移1位=0xD0） */

#define MPU6050_REG_WHO_AM_I    0x75    /* 芯片ID寄存器 */
#define MPU6050_REG_PWR_MGMT_1  0x6B    /* 电源管理1 */
#define MPU6050_REG_PWR_MGMT_2  0x6C    /* 电源管理2 */
#define MPU6050_REG_SMPLRT_DIV  0x19    /* 采样率分频 */
#define MPU6050_REG_CONFIG      0x1A    /* 配置寄存器 */
#define MPU6050_REG_GYRO_CONFIG 0x1B    /* 陀螺仪配置 */
#define MPU6050_REG_ACCEL_CONFIG 0x1C   /* 加速度计配置 */

#define MPU6050_REG_ACCEL_XOUT_H 0x3B   /* 加速度X高字节 */
#define MPU6050_REG_GYRO_XOUT_H  0x43   /* 陀螺仪X高字节 */
#define MPU6050_REG_TEMP_OUT_H   0x41   /* 温度高字节 */

/* ==================== 数据结构 ==================== */
typedef struct {
    int16_t accel_x;    /* 加速度原始值 */
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;     /* 陀螺仪原始值 */
    int16_t gyro_y;
    int16_t gyro_z;

    float   yaw;        /* 偏航角（积分得到，单位：度） */
    float   pitch;      /* 俯仰角 */
    float   roll;       /* 翻滚角 */

    float   gyro_z_dps; /* Z轴角速度（度/秒） */
} MPU6050_DataTypeDef;

/* ==================== 函数声明 ==================== */

uint8_t MPU6050_Init(void);
void MPU6050_ReadAll(MPU6050_DataTypeDef *data);
void MPU6050_UpdateYaw(MPU6050_DataTypeDef *data, float dt);
void MPU6050_ResetYaw(MPU6050_DataTypeDef *data);
float MPU6050_GetYaw(MPU6050_DataTypeDef *data);

#endif /* __MPU6050_H */
