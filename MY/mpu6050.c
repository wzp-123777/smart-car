#include "mpu6050.h"

volatile MPU6050_DataTypeDef g_mpu_data_uart = {0};
static uint8_t rx_buf[11];
static uint8_t rx_index = 0;

void MPU6050_Init(void) { }

void MPU6050_ReadAll(MPU6050_DataTypeDef *data) {
    __disable_irq();
    data->accel_x = g_mpu_data_uart.accel_x;
    data->accel_y = g_mpu_data_uart.accel_y;
    data->accel_z = g_mpu_data_uart.accel_z;
    data->gyro_x = g_mpu_data_uart.gyro_x;
    data->gyro_y = g_mpu_data_uart.gyro_y;
    data->gyro_z = g_mpu_data_uart.gyro_z;
    data->gyro_z_dps = g_mpu_data_uart.gyro_z_dps;
    data->yaw = g_mpu_data_uart.yaw;
    __enable_irq();
}

void MPU6050_UpdateYaw(MPU6050_DataTypeDef *data, float dt) { }

void MPU6050_ResetYaw(MPU6050_DataTypeDef *data) {
    __disable_irq();
    data->yaw = 0.0f;
    g_mpu_data_uart.yaw = 0.0f;
    __enable_irq();
}

void MPU6050_UartParse(uint8_t res) {
    uint8_t checksum = 0;
    uint8_t i = 0;

    if (rx_index == 0 && res != 0x55) {
        return;
    }

    rx_buf[rx_index] = res;
    rx_index++;

    if (rx_index < sizeof(rx_buf)) {
        return;
    }

    for (i = 0; i < 10; i++) {
        checksum += rx_buf[i];
    }

    if (checksum == rx_buf[10]) {
        if (rx_buf[1] == 0x51) {
            g_mpu_data_uart.accel_x = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
            g_mpu_data_uart.accel_y = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
            g_mpu_data_uart.accel_z = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
        }
        else if (rx_buf[1] == 0x52) {
            g_mpu_data_uart.gyro_x = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
            g_mpu_data_uart.gyro_y = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
            g_mpu_data_uart.gyro_z = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
            g_mpu_data_uart.gyro_z_dps = (float)g_mpu_data_uart.gyro_z / 32768.0f * 2000.0f;
        }
        else if (rx_buf[1] == 0x53) {
            int16_t raw_yaw = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
            g_mpu_data_uart.yaw = (float)raw_yaw / 32768.0f * 180.0f;
        }
    }

    rx_index = 0;
}

float MPU6050_GetYaw(MPU6050_DataTypeDef *data) { return data->yaw; }
