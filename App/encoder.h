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

#include "stm32f4xx_hal.h"

/* ==================== 函数声明 ==================== */

/**
 * @brief  编码器定时器初始化（编码器接口模式）
 * @param  htim: 定时器句柄（如 &htim2, &htim3）
 * @note   需要在CubeMX中将定时器配置为 Encoder Mode
 *         或者手动调用此函数初始化
 */
void Encoder_Init(TIM_HandleTypeDef *htim);

/**
 * @brief  读取编码器计数值并清零（每10ms调用一次）
 * @param  htim: 定时器句柄
 * @retval 有符号计数值（正=正转，负=反转）
 * @note   此值代表10ms内的编码器脉冲数，即当前转速
 */
int16_t Encoder_Read(TIM_HandleTypeDef *htim);

#endif /* __ENCODER_H */
