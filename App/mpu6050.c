/**
 * @file    mpu6050.c
 * @brief   MPU6050 陀螺仪驱动实现
 */

#include "mpu6050.h"

extern void delay_ms(__IO uint32_t nTime);

/* 陀螺仪灵敏度（±250°/s 对应 131 LSB/°/s） */
#define GYRO_SENSITIVITY    131.0f

/* Z轴零偏补偿值（需要校准，静止时陀螺仪Z轴的平均值） */
static float s_gyro_z_offset = 0.0f;

/* 注意：此处使用简单的阻塞轮询I2C。如果在实际应用中卡死，建议加入超时机制 */
static void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    I2C_SendData(I2C1, reg);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    I2C_SendData(I2C1, data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    I2C_GenerateSTOP(I2C1, ENABLE);
}

static void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    I2C_SendData(I2C1, reg);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    I2C_Send7bitAddress(I2C1, MPU6050_ADDR, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    while(len)
    {
        if(len == 1)
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        *buf = I2C_ReceiveData(I2C1);
        buf++;
        len--;
    }
    
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i = 0;
    uint16_t i;
    int32_t gyro_z_sum = 0;
    uint8_t raw[14];

    MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x68)
    {
        return 1;
    }

    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80);
    delay_ms(100);

    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01);
    delay_ms(10);

    MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03);
    MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00);
    MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00);
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_2, 0x00);

    delay_ms(200);

    for (i = 0; i < 200; i++)
    {
        MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, raw, 14);
        int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
        gyro_z_sum += gz;
        delay_ms(5);
    }
    s_gyro_z_offset = (float)gyro_z_sum / 200.0f;

    return 0;
}

void MPU6050_ReadAll(MPU6050_DataTypeDef *data)
{
    uint8_t raw[14];

    MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, raw, 14);

    data->accel_x = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t)((raw[4] << 8) | raw[5]);

    data->gyro_x = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_y = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_z = (int16_t)((raw[12] << 8) | raw[13]);

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
