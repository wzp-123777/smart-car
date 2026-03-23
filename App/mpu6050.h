#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32f4xx.h"

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float gyro_z_dps;
    float yaw;
} MPU6050_DataTypeDef;

extern volatile MPU6050_DataTypeDef g_mpu_data_uart;

void MPU6050_Init(void);
void MPU6050_ReadAll(MPU6050_DataTypeDef *data);
void MPU6050_UpdateYaw(MPU6050_DataTypeDef *data, float dt);
void MPU6050_ResetYaw(MPU6050_DataTypeDef *data);
void MPU6050_UartParse(uint8_t res);
#endif

float MPU6050_GetYaw(MPU6050_DataTypeDef *data);
