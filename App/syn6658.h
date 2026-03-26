/**
 * @file    syn6658.h
 * @brief   SYN6658 语音合成模块驱动
 * @note    通过 UART2 与 STM32 通信，波特率 9600
 *          支持 GB2312 中文编码和 Unicode 编码
 * 
 * 【接线】
 *  SYN6658 TX → STM32 UART2_RX (PA3)  （可选，用于接收忙状态）
 *  SYN6658 RX → STM32 UART2_TX (PA2)
 *  SYN6658 BUSY → 可接GPIO判断是否播报完成（可选）
 */
#ifndef __SYN6658_H
#define __SYN6658_H

#include "stm32f4xx.h"
#include <string.h>

/* ==================== 协议定义 ==================== */
#define SYN_HEADER_H        0xFD    /* 帧头高字节 */
#define SYN_ENCODING_GB2312 0x00    /* GB2312 编码 */
#define SYN_ENCODING_GBK    0x01    /* GBK 编码 */
#define SYN_ENCODING_BIG5   0x02    /* BIG5 编码 */
#define SYN_ENCODING_UNI    0x03    /* Unicode 编码 */

#define SYN_CMD_PLAY        0x01    /* 合成播放命令 */
#define SYN_CMD_STOP        0x02    /* 停止 */
#define SYN_CMD_PAUSE       0x03    /* 暂停 */
#define SYN_CMD_RESUME      0x04    /* 恢复 */
#define SYN_CMD_STATUS      0x21    /* 查询状态 */
#define SYN_CMD_POWERDOWN   0x88    /* 低功耗 */

/* ==================== 函数声明 ==================== */

/**
 * @brief  SYN6658 初始化
 */
void SYN6658_Init(void);

/**
 * @brief  发送文本进行语音合成（GB2312编码）
 * @param  text: 要播报的中文文本（GB2312编码字符串）
 * @note   在Keil中，中文字符串默认为GB2312编码
 *         例如: SYN6658_Speak("检测到圆形");
 */
void SYN6658_Speak(const char *text);

/**
 * @brief  发送带音量/语速控制的语音
 * @param  text: 文本
 * @param  volume: 音量 0~16
 * @param  speed: 语速 0~10
 */
void SYN6658_SpeakEx(const char *text,
                     uint8_t volume, uint8_t speed);

/**
 * @brief  停止当前播报
 */
void SYN6658_Stop(void);

/**
 * @brief  暂停播报
 */
void SYN6658_Pause(void);

/**
 * @brief  恢复播报
 */
void SYN6658_Resume(void);

/**
 * @brief  根据物体ID播报对应名称
 * @param  object_id: 物体识别ID
 */
void SYN6658_ReportObject(uint8_t object_id);

/**
 * @brief  播报巡检点位信息
 * @param  point_num: 巡检点编号 (1,2,3...)
 */
void SYN6658_ReportPoint(uint8_t point_num);

/**
 * @brief  播报开始运行提示
 */
void SYN6658_ReportStartup(void);

/**
 * @brief  播报碰撞停机提示
 */
void SYN6658_ReportCollision(void);

/**
 * @brief  播报到达A/B/C/D点
 * @param  point: 'A', 'B', 'C', 'D'
 */
void SYN6658_ReportStation(char point);

#endif /* __SYN6658_H */
