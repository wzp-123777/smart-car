#ifndef __INFRARED_H
#define __INFRARED_H

#include "stm32f4xx.h"

/* ==================== 引脚定义 ==================== */
#define IR1_PORT    GPIOD
#define IR1_PIN     GPIO_Pin_1    /* 最左 */
#define IR2_PORT    GPIOD
#define IR2_PIN     GPIO_Pin_2
#define IR3_PORT    GPIOD
#define IR3_PIN     GPIO_Pin_3    /* 中间 */
#define IR4_PORT    GPIOD
#define IR4_PIN     GPIO_Pin_4
#define IR5_PORT    GPIOD
#define IR5_PIN     GPIO_Pin_5    /* 最右 */

/* 传感器数量 */
#define IR_SENSOR_COUNT     5

/* ==================== 数据结构 ==================== */
typedef struct {
    uint8_t sensor[IR_SENSOR_COUNT];
    uint8_t raw_byte;
    int8_t  position;
    uint8_t all_black;
    uint8_t all_white;
} IR_DataTypeDef;

void IR_Init(void);
void IR_Read(IR_DataTypeDef *ir_data);
int8_t IR_GetPosition(IR_DataTypeDef *ir_data);
void IR_ResetTracking(void);

#endif
