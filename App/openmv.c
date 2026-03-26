/**
 * @file    openmv.c
 * @brief   STM32 与 OpenMV 串口通信实现
 * @note    OpenMV 当前常在线主动上报，STM32 侧只保留接收解析逻辑。
 *          OpenMV_SendCmd() 保留为空实现，兼容历史调用点。
 */

#include "openmv.h"

/* ==================== 全局变量 ==================== */
volatile OpenMV_DataTypeDef g_openmv_data = {0};
uint8_t g_openmv_rx_buf[OPENMV_RX_BUF_SIZE] = {0};
uint8_t g_openmv_rx_index = 0;

/* 接收状态机 */
static uint8_t s_rx_state = 0;
static uint8_t s_rx_temp[8] = {0};
static uint8_t s_rx_cnt = 0;

/**
 * @brief  初始化OpenMV通信
 */
void OpenMV_Init(void)
{
    s_rx_state = 0;
    s_rx_cnt = 0;
    g_openmv_data.is_valid = 0;
    g_openmv_data.is_new = 0;
}

/**
 * @brief  兼容旧接口的空函数
 * @note   当前 OpenMV 常在线主动上报，本函数不再发送任何数据。
 */
void OpenMV_SendCmd(uint8_t cmd)
{
    (void)cmd;
}

/**
 * @brief  逐字节解析OpenMV返回的数据（状态机方式）
 * @note   协议: [0xBB] [ID] [Xh] [Xl] [Yh] [Yl] [校验] [0x55]
 *         在串口接收回调函数中逐字节调用
 *
 * 状态机流程:
 *   状态0: 等待帧头 0xBB
 *   状态1: 接收数据 (6字节: ID + Xh + Xl + Yh + Yl + 校验)
 *   状态2: 等待帧尾 0x55，校验通过则更新数据
 */
void OpenMV_ParseByte(uint8_t byte)
{
    switch (s_rx_state)
    {
    case 0:  /* 等待帧头 */
        if (byte == OPENMV_RX_HEADER)
        {
            s_rx_state = 1;
            s_rx_cnt = 0;
        }
        break;

    case 1:  /* 接收数据域 */
        s_rx_temp[s_rx_cnt++] = byte;
        if (s_rx_cnt >= 6)  /* ID(1) + X(2) + Y(2) + 校验(1) = 6字节 */
        {
            s_rx_state = 2;
        }
        break;

    case 2:  /* 等待帧尾 */
        if (byte == OPENMV_RX_TAIL)
        {
            uint8_t checksum = 0;
            uint8_t i;

            for (i = 0; i < 5; i++)  /* 对前5个字节求和 */
            {
                checksum += s_rx_temp[i];
            }
            checksum &= 0xFF;

            if (checksum == s_rx_temp[5])
            {
                g_openmv_data.object_id = s_rx_temp[0];
                g_openmv_data.pos_x = ((uint16_t)s_rx_temp[1] << 8) | s_rx_temp[2];
                g_openmv_data.pos_y = ((uint16_t)s_rx_temp[3] << 8) | s_rx_temp[4];
                g_openmv_data.is_valid = 1;
                g_openmv_data.is_new = 1;
            }
        }

        s_rx_state = 0;
        s_rx_cnt = 0;
        break;

    default:
        s_rx_state = 0;
        s_rx_cnt = 0;
        break;
    }
}

/**
 * @brief  获取最新识别结果
 */
OpenMV_DataTypeDef OpenMV_GetResult(void)
{
    OpenMV_DataTypeDef snapshot;

    __disable_irq();
    snapshot.object_id = g_openmv_data.object_id;
    snapshot.pos_x = g_openmv_data.pos_x;
    snapshot.pos_y = g_openmv_data.pos_y;
    snapshot.is_valid = g_openmv_data.is_valid;
    snapshot.is_new = g_openmv_data.is_new;
    __enable_irq();

    return snapshot;
}

/**
 * @brief  检查是否有新数据
 */
uint8_t OpenMV_HasNewData(void)
{
    uint8_t is_new;

    __disable_irq();
    is_new = g_openmv_data.is_new;
    __enable_irq();

    return is_new;
}

/**
 * @brief  清除新数据标志
 */
void OpenMV_ClearNewFlag(void)
{
    __disable_irq();
    g_openmv_data.is_new = 0;
    __enable_irq();
}
