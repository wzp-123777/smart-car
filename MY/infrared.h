/**
 * @file    infrared.h
 * @brief   5路红外巡线传感器模块
 * @note    传感器从左到右编号: IR1(最左) ~ IR5(最右)
 *          检测到黑线=1, 未检测到=0
 * 
 * 【引脚分配建议】（为避开MPU6050引脚，已改到 GPIOD）
 *  IR1(最左): PD1
 *  IR2:       PD2
 *  IR3(中间): PD3
 *  IR4:       PD4
 *  IR5(最右): PD5
 */
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
    uint8_t sensor[IR_SENSOR_COUNT];  // sensor[0]=最左, sensor[4]=最右
    uint8_t raw_byte;                  // 5位二进制原始值（用于快速查表）
    int8_t  position;                  // 偏移量: -2(偏左) ~ +2(偏右), 0=居中
    uint8_t all_black;                 // 全部检测到黑线（十字路口/T字路口）
    uint8_t all_white;                 // 全部未检测到（脱线）
} IR_DataTypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  红外传感器GPIO初始化
 */
void IR_Init(void);

/**
 * @brief  读取5路红外传感器状态
 * @param  ir_data: 数据结构指针
 */
void IR_Read(IR_DataTypeDef *ir_data);

/**
 * @brief  计算小车相对黑线的偏移量（用于PID纠偏）
 * @param  ir_data: 传感器数据
 * @retval 偏移量 -2 ~ +2，0表示居中
 * @note   使用加权平均法计算位置
 */
int8_t IR_GetPosition(IR_DataTypeDef *ir_data);

#endif /* __INFRARED_H */
