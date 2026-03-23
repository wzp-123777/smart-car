/**
 * @file    infrared.c
 * @brief   5路红外巡线传感器实现
 * @note    加权平均法计算偏移量，供转向PID使用
 */

#include "infrared.h"

/**
 * @brief  红外传感器GPIO初始化
 */
void IR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* 配置 PD1~PD5 为上拉输入 */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3        
                               | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
 * @brief  读取5路传感器原始状态
 * @note   检测到黑线=1, 白色地面=0
 *         如果传感器输出是反逻辑，在此处取反即可
 */
void IR_Read(IR_DataTypeDef *ir_data)
{
    /* 读取各路传感器（根据传感器极性决定是否取反） */
    ir_data->sensor[0] = !GPIO_ReadInputDataBit(IR1_PORT, IR1_PIN);  /* 最左 */
    ir_data->sensor[1] = !GPIO_ReadInputDataBit(IR2_PORT, IR2_PIN);
    ir_data->sensor[2] = !GPIO_ReadInputDataBit(IR3_PORT, IR3_PIN);  /* 中间 */
    ir_data->sensor[3] = !GPIO_ReadInputDataBit(IR4_PORT, IR4_PIN);
    ir_data->sensor[4] = !GPIO_ReadInputDataBit(IR5_PORT, IR5_PIN);  /* 最右 */

    /* 合成原始字节（方便调试和查表） */
    ir_data->raw_byte = (ir_data->sensor[0] << 4)
                      | (ir_data->sensor[1] << 3)
                      | (ir_data->sensor[2] << 2)
                      | (ir_data->sensor[3] << 1)
                      | (ir_data->sensor[4] << 0);

    /* 判断特殊状态 */
    ir_data->all_black = (ir_data->raw_byte == 0x1F) ? 1 : 0;  /* 11111 全黑 */
    ir_data->all_white = (ir_data->raw_byte == 0x00) ? 1 : 0;  /* 00000 全白 */

    /* 计算偏移量 */
    ir_data->position = IR_GetPosition(ir_data);
}

/**
 * @brief  加权平均法计算偏移量
 * @note   权重: 最左=-2, 左=-1, 中=0, 右=+1, 最右=+2
 *         偏移量 = 加权和 / 检测到黑线的传感器数量
 * 
 * 示例：
 *   10000 → position = -2（黑线在最左边，小车偏右了，需要左转）
 *   01100 → position = -0.5 ≈ -1（黑线偏左）
 *   00100 → position = 0（居中）
 *   00011 → position = +1.5 ≈ +2（黑线偏右）
 *   00001 → position = +2（黑线在最右边，小车偏左了，需要右转）
 */
int8_t IR_GetPosition(IR_DataTypeDef *ir_data)
{
    int16_t weighted_sum = 0;
    uint8_t active_count = 0;
    int8_t  weights[5] = {2, 1, 0, -1, -2};  /* 从左到右的权重 */
    uint8_t i;

    for (i = 0; i < IR_SENSOR_COUNT; i++)
    {
        if (ir_data->sensor[i])
        {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    /* 没有检测到黑线，返回0（保持上一次方向，由状态机处理） */
    if (active_count == 0)
        return 0;

    /* 计算加权平均并返回 */
    return (int8_t)(weighted_sum / (int16_t)active_count);
}




