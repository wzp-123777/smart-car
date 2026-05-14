/**
 * @file    openmv.h
 * @brief   STM32 与 OpenMV H7 Plus 串口通信模块
 * @note    OpenMV 侧由 STM32 按任务检测点启动。
 *          STM32 通过 UART1 发送命令帧控制左/右检测窗口，OpenMV 只在
 *          检测窗口内识别并主动上报结果。
 *
 *          通信协议：
 *          STM32命令: [帧头0xAA] [命令] [帧尾0x55]
 *          OpenMV返回: [帧头0xBB] [物体ID] [X坐标高] [X坐标低] [Y坐标高] [Y坐标低] [校验和] [帧尾0x55]
 *
 *          使用 UART1 通信，波特率 115200
 *          引脚: PB6(TX) / PB7(RX)，避免与TIM1(PA8~PA11)冲突
 */
#ifndef __OPENMV_H
#define __OPENMV_H

#include "stm32f4xx.h"
#include <string.h>

/* ==================== 通信协议定义 ==================== */

/* OpenMV返回给STM32的数据 */
#define OPENMV_RX_HEADER        0xBB    /* 返回帧头 */
#define OPENMV_RX_TAIL          0x55    /* 返回帧尾 */
#define OPENMV_RX_BUF_SIZE      16      /* 接收缓冲区大小 */

#define OPENMV_TX_HEADER        0xAA    /* STM32命令帧头 */
#define OPENMV_TX_TAIL          0x55    /* STM32命令帧尾 */
#define OPENMV_CMD_STOP         0x00    /* 停止识别/回到空闲 */
#define OPENMV_CMD_DETECT_LEFT  0x11    /* 启动检测，云台偏左 */
#define OPENMV_CMD_DETECT_RIGHT 0x12    /* 启动检测，云台偏右 */

/* 物体ID定义（与 OpenMV/main.py 保持一致） */
#define OBJ_NONE                0x00    /* 未识别到 */
#define OBJ_LIGHTER             0x01    /* 打火机 */
#define OBJ_SCISSORS            0x02    /* 剪刀 */
#define OBJ_HAMMER              0x03    /* 锤子 */

/* 兼容旧命名，避免其他文件引用时出错 */
#define OBJ_CIRCLE              OBJ_LIGHTER
#define OBJ_TRIANGLE            OBJ_SCISSORS
#define OBJ_SQUARE              OBJ_HAMMER

/* ==================== 数据结构 ==================== */
typedef struct {
    uint8_t  object_id;     /* 识别到的物体ID */
    uint16_t pos_x;         /* 物体中心X坐标 */
    uint16_t pos_y;         /* 物体中心Y坐标 */
    uint8_t  is_valid;      /* 数据是否有效（1=有效，0=无效） */
    uint8_t  is_new;        /* 是否有新数据（1=新数据未处理） */
} OpenMV_DataTypeDef;

/* ==================== 全局变量（extern） ==================== */
extern volatile OpenMV_DataTypeDef g_openmv_data;
extern uint8_t g_openmv_rx_buf[OPENMV_RX_BUF_SIZE];
extern uint8_t g_openmv_rx_index;
extern volatile uint32_t g_openmv_rx_byte_count;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化OpenMV通信（启动串口接收中断）
 */
void OpenMV_Init(void);

/**
 * @brief  发送OpenMV任务命令
 * @param  cmd: OPENMV_CMD_STOP / OPENMV_CMD_DETECT_LEFT / OPENMV_CMD_DETECT_RIGHT
 */
void OpenMV_SendCmd(uint8_t cmd);

/**
 * @brief  解析OpenMV返回数据（在串口接收中断回调中调用）
 * @param  byte: 接收到的单个字节
 */
void OpenMV_ParseByte(uint8_t byte);

/**
 * @brief  获取最新的识别结果
 * @retval OpenMV_DataTypeDef 数据结构
 */
OpenMV_DataTypeDef OpenMV_GetResult(void);

/**
 * @brief  检查是否有新的识别结果
 * @retval 1=有新数据, 0=无
 */
uint8_t OpenMV_HasNewData(void);

/**
 * @brief  清除新数据标志
 */
void OpenMV_ClearNewFlag(void);

/**
 * @brief  清空当前识别结果，避免检测窗口复用旧结果
 */
void OpenMV_ResetResult(void);

/**
 * @brief  获取当前累计接收到的 OpenMV 串口字节数
 * @retval 字节计数
 */
uint32_t OpenMV_GetRxByteCount(void);

/* Current Task3 model labels: background / hammer / lighter / scissors */
#define OPENMV_LABEL_BACKGROUND "background"
#define OPENMV_LABEL_HAMMER     "hammer"
#define OPENMV_LABEL_LIGHTER    "lighter"
#define OPENMV_LABEL_SCISSORS   "scissors"
#define OPENMV_LABEL_SCISSOR    "scissor"

#endif /* __OPENMV_H */
