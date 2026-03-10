/**
 * @file    encoder.c
 * @brief   编码器测速实现
 * @note    使用STM32定时器的编码器接口模式，硬件计数
 *          每10ms读取一次计数器值作为速度反馈
 */

#include "encoder.h"

/**
 * @brief  编码器定时器初始化
 * @note   如果已在CubeMX中配置为Encoder模式，只需调用HAL_TIM_Encoder_Start
 */
void Encoder_Init(TIM_HandleTypeDef *htim)
{
    /* 启动编码器接口模式，同时捕获CH1和CH2 */
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

    /* 清零计数器 */
    __HAL_TIM_SET_COUNTER(htim, 0);
}

/**
 * @brief  读取编码器计数值并清零
 * @note   每10ms在定时器中断中调用一次
 *         返回值直接作为PID的实际速度输入
 * 
 * 原理说明：
 *   定时器在编码器模式下自动对A/B相脉冲计数
 *   读取后清零，得到的就是10ms内的脉冲增量
 *   正值=正转，负值=反转
 *   计数器为16位有符号（利用溢出特性自动处理方向）
 */
int16_t Encoder_Read(TIM_HandleTypeDef *htim)
{
    int16_t count;

    /* 读取当前计数值（自动处理有符号） */
    count = (int16_t)__HAL_TIM_GET_COUNTER(htim);

    /* 读取后立即清零，下次中断时得到的是增量值 */
    __HAL_TIM_SET_COUNTER(htim, 0);

    return count;
}
