/**
 * @file    openmv.c
 * @brief   STM32 与 OpenMV 串口通信实现
 * @note    接收使用逐字节中断+状态机解析，发送使用阻塞式
 * 
 * 【OpenMV端配套代码】（需要在OpenMV IDE中烧写）
 * 
 *   import sensor, image, time, struct
 *   from pyb import UART
 *   
 *   uart = UART(3, 115200)  # OpenMV的UART3对应P4(TX)/P5(RX)
 *   
 *   while True:
 *       if uart.any():
 *           data = uart.read(3)  # 读取3字节指令
 *           if data and data[0] == 0xAA and data[2] == 0x55:
 *               cmd = data[1]
 *               if cmd == 0x01:  # 检测指令
 *                   img = sensor.snapshot()
 *                   # ... 你的识别算法 ...
 *                   obj_id = 0x01  # 识别结果
 *                   cx, cy = 160, 120  # 中心坐标
 *                   checksum = (obj_id + (cx>>8) + (cx&0xFF) + (cy>>8) + (cy&0xFF)) & 0xFF
 *                   uart.write(struct.pack('BBBBBBB', 0xBB, obj_id, cx>>8, cx&0xFF, cy>>8, cy&0xFF, checksum, 0x55))
 */

#include "openmv.h"

/* ==================== 全局变量 ==================== */
OpenMV_DataTypeDef g_openmv_data = {0};
uint8_t g_openmv_rx_buf[OPENMV_RX_BUF_SIZE] = {0};
uint8_t g_openmv_rx_index = 0;

/* 接收状态机 */
static uint8_t s_rx_state = 0;
static uint8_t s_rx_temp[8] = {0};
static uint8_t s_rx_cnt = 0;

/* 单字节接收缓冲 */
static uint8_t s_uart_rx_byte = 0;
static UART_HandleTypeDef *s_openmv_uart = NULL;

/**
 * @brief  初始化OpenMV通信
 */
void OpenMV_Init(UART_HandleTypeDef *huart)
{
    s_openmv_uart = huart;
    s_rx_state = 0;
    s_rx_cnt = 0;
    g_openmv_data.is_valid = 0;
    g_openmv_data.is_new = 0;

    /* 启动串口接收中断（逐字节接收） */
    HAL_UART_Receive_IT(huart, &s_uart_rx_byte, 1);
}

/**
 * @brief  向OpenMV发送指令
 * @note   协议: [0xAA] [CMD] [校验和] [0x55]
 */
void OpenMV_SendCmd(UART_HandleTypeDef *huart, uint8_t cmd)
{
    uint8_t tx_buf[4];

    tx_buf[0] = OPENMV_CMD_HEADER;  /* 帧头 0xAA */
    tx_buf[1] = cmd;                 /* 命令字节 */
    tx_buf[2] = cmd;                 /* 校验和（简单起见=命令本身） */
    tx_buf[3] = OPENMV_CMD_TAIL;    /* 帧尾 0x55 */

    HAL_UART_Transmit(huart, tx_buf, 4, 100);
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
            /* 计算校验和 */
            uint8_t checksum = 0;
            uint8_t i;
            for (i = 0; i < 5; i++)  /* 对前5个字节求和 */
            {
                checksum += s_rx_temp[i];
            }
            checksum &= 0xFF;

            /* 校验通过，更新数据 */
            if (checksum == s_rx_temp[5])
            {
                g_openmv_data.object_id = s_rx_temp[0];
                g_openmv_data.pos_x = ((uint16_t)s_rx_temp[1] << 8) | s_rx_temp[2];
                g_openmv_data.pos_y = ((uint16_t)s_rx_temp[3] << 8) | s_rx_temp[4];
                g_openmv_data.is_valid = 1;
                g_openmv_data.is_new = 1;
            }
        }
        /* 无论帧尾是否正确，都回到状态0 */
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
    return g_openmv_data;
}

/**
 * @brief  检查是否有新数据
 */
uint8_t OpenMV_HasNewData(void)
{
    return g_openmv_data.is_new;
}

/**
 * @brief  清除新数据标志
 */
void OpenMV_ClearNewFlag(void)
{
    g_openmv_data.is_new = 0;
}

/**
 * @brief  串口接收完成回调（在 stm32f4xx_it.c 或 main.c 中被HAL库自动调用）
 * @note   所有UART的接收完成都会进入此回调，需要判断是哪个UART
 *         此函数已在本模块中实现，如果你的工程有其它UART也用中断接收，
 *         请将此函数移到统一的回调管理处
 */
void OpenMV_UART_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_openmv_uart)
    {
        /* 将接收到的字节送入解析器 */
        OpenMV_ParseByte(s_uart_rx_byte);

        /* 重新启动接收中断（必须！） */
        HAL_UART_Receive_IT(s_openmv_uart, &s_uart_rx_byte, 1);
    }
}
