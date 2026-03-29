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

    /* 配置 PD1~PD5 为数字输入；开启上拉可降低悬空抖动导致的误判概率 */
    GPIO_InitStruct.GPIO_Pin   = IR1_PIN | IR2_PIN | IR3_PIN | IR4_PIN | IR5_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
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
 * @brief  计算当前黑线相对车体中心的位置
 * @note
 *   1. 权重从左到右依次为 {-20, -10, 0, 10, 20}，比旧版更细腻。
 *   2. 当 5 路传感器全部为白色时，直接返回 0，不做丢线方向记忆。
 *   3. 正常压线时直接做整数平均，依赖 C 语言默认的向零取整特性，
 *      计算简单、执行开销小。
 */
int8_t IR_GetPosition(IR_DataTypeDef *ir_data)
{
    static const int8_t weights[IR_SENSOR_COUNT] = {-20, -10, 0, 10, 20};
    int16_t weighted_sum = 0;
    uint8_t active_count = 0;
    int8_t current_position;
    uint8_t i;

    for (i = 0; i < IR_SENSOR_COUNT; i++)
    {
        if (ir_data->sensor[i] != 0U)
        {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    /* 全白丢线：不做方向记忆，直接返回 0。 */
    if (active_count == 0U)
    {
        return 0;
    }

    /* 正常检测到黑线时，直接整除得到平滑位置值。 */
    current_position = (int8_t)(weighted_sum / (int16_t)active_count);
    return current_position;
}

void IR_ResetTracking(void)
{
    /* 丢线记忆已关闭，保留空函数兼容旧调用。 */
}








