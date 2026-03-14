/**
 * @file    encoder.h
 * @brief   编码器测速模块 —— 使用定时器编码器模式
 * @note    STM32F407 的 TIM2/TIM3/TIM4/TIM5 支持编码器接口模式
 * 
 * 【引脚分配建议】
 *  左侧编码器: TIM2 (PA0=CH1, PA1=CH2) 或 TIM4 (PB6=CH1, PB7=CH2)
 *  右侧编码器: TIM3 (PA6=CH1, PA7=CH2) 或 TIM5 (PA0=CH1, PA1=CH2)
 *  注意：根据实际接线选择，避免引脚冲突
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h"

/* ==================== 函数声明 ==================== */

int16_t Encoder_Read_TIM2(void);
int16_t Encoder_Read_TIM3(void);

#endif /* __ENCODER_H */
