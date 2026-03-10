/**
 * @file    mpu6050.c
 * @brief   MPU6050 陀螺仪驱动实现
 * @note    简单积分法计算偏航角，精度足够辅助转弯
 *          如需更高精度，可替换为互补滤波或卡尔曼滤波
 */

#include "mpu6050.h"

/* 陀螺仪灵敏度（±250°/s 对应 131 LSB/°/s） */
#define GYRO_SENSITIVITY    131.0f

/* Z轴零偏补偿值（需要校准，静止时陀螺仪Z轴的平均值） */
static float s_gyro_z_offset = 0.0f;

/**
 * @brief  写一个字节到MPU6050寄存器
 */
static void MPU6050_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                      &data, 1, 100);
}

/**
 * @brief  从MPU6050读取多个字节
 */
static void MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                     buf, len, 100);
}

/**
 * @brief  MPU6050 初始化
 * @retval 0=成功
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0;
    uint16_t i;
    int32_t gyro_z_sum = 0;
    uint8_t raw[14];

    /* 读取WHO_AM_I，确认芯片存在 */
    MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x68)
    {
        return 1;  /* 芯片未检测到 */
    }

    /* 复位芯片 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 唤醒，选择X轴陀螺仪作为时钟源 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* 采样率 = 1kHz / (1+9) = 100Hz */
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x09);

    /* 低通滤波器 带宽44Hz */
    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, 0x03);

    /* 陀螺仪量程 ±250°/s */
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00);

    /* 加速度计量程 ±2g */
    MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00);

    /* 使能所有轴 */
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_2, 0x00);

    HAL_Delay(200);

    /* === 零偏校准：静止状态下采集200次Z轴陀螺仪数据取平均 === */
    for (i = 0; i < 200; i++)
    {
        MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, raw, 14);
        int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
        gyro_z_sum += gz;
        HAL_Delay(5);
    }
    s_gyro_z_offset = (float)gyro_z_sum / 200.0f;

    return 0;
}

/**
 * @brief  读取所有原始数据
 * @note   一次读14字节: AX(2) + AY(2) + AZ(2) + TEMP(2) + GX(2) + GY(2) + GZ(2)
 */
void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_DataTypeDef *data)
{
    uint8_t raw[14];

    MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, raw, 14);

    /* 解析加速度（高字节在前） */
    data->accel_x = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t)((raw[4] << 8) | raw[5]);

    /* 解析陀螺仪 */
    data->gyro_x = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_y = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_z = (int16_t)((raw[12] << 8) | raw[13]);

    /* 计算Z轴角速度（减去零偏，转换为度/秒） */
    data->gyro_z_dps = ((float)data->gyro_z - s_gyro_z_offset) / GYRO_SENSITIVITY;
}

/**
 * @brief  更新偏航角（简单积分）
 * @param  dt: 采样周期，单位秒（10ms = 0.01f）
 * @note   yaw += gyro_z_dps * dt
 *         滤除微小噪声（死区 ±0.5°/s）
 */
void MPU6050_UpdateYaw(MPU6050_DataTypeDef *data, float dt)
{
    /* 死区滤波：角速度小于0.5°/s认为是噪声 */
    if (data->gyro_z_dps > 0.5f || data->gyro_z_dps < -0.5f)
    {
        data->yaw += data->gyro_z_dps * dt;
    }

    /* 限制范围 -180 ~ 180 */
    if (data->yaw > 180.0f)  data->yaw -= 360.0f;
    if (data->yaw < -180.0f) data->yaw += 360.0f;
}

/**
 * @brief  重置偏航角
 */
void MPU6050_ResetYaw(MPU6050_DataTypeDef *data)
{
    data->yaw = 0.0f;
}

/**
 * @brief  获取当前偏航角
 */
float MPU6050_GetYaw(MPU6050_DataTypeDef *data)
{
    return data->yaw;
}
