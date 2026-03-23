/**
 * @file    openmv.h
 * @brief   STM32 与 OpenMV H7 Plus 串口通信模块
 * @note    当前仓库的 OpenMV/main.py 采用主动识别、主动上报模式。
 *          本文件仍保留命令帧定义，便于后续切回按需触发。
 *
 *          通信协议：
 *          STM32 发送: [帧头0xAA] [命令字节] [校验和] [帧尾0x55]
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

/* STM32发送给OpenMV的指令 */
#define OPENMV_CMD_HEADER       0xAA    /* 帧头 */
#define OPENMV_CMD_TAIL         0x55    /* 帧尾 */
#define OPENMV_CMD_DETECT       0x01    /* 识别指令：开始检测 */
#define OPENMV_CMD_STOP         0x02    /* 停止检测 */
#define OPENMV_CMD_COLOR        0x03    /* 颜色识别模式 */
#define OPENMV_CMD_SHAPE        0x04    /* 形状识别模式 */
#define OPENMV_CMD_QRCODE       0x05    /* 二维码识别模式 */

/* OpenMV返回给STM32的数据 */
#define OPENMV_RX_HEADER        0xBB    /* 返回帧头 */
#define OPENMV_RX_TAIL          0x55    /* 返回帧尾 */
#define OPENMV_RX_BUF_SIZE      16      /* 接收缓冲区大小 */

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

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化OpenMV通信（启动串口接收中断）
 */
void OpenMV_Init(void);

/**
 * @brief  向OpenMV发送识别指令
 * @param  cmd: 命令字节（如 OPENMV_CMD_DETECT）
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

#endif /* __OPENMV_H */
