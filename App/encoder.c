/**
 * @file    encoder.c
 * @brief   编码器测速实现 (标准库版)
 */

#include "encoder.h"

int16_t Encoder_Read_TIM2(void)
{
    int16_t count;
    count = (int16_t)TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2, 0);
    return -count; // 左侧编码器需要取反，消除PID正反馈
}

int16_t Encoder_Read_TIM3(void)
{
    int16_t count;
    count = (int16_t)TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);
    return count;
}
