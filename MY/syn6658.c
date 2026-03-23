/**
 * @file    syn6658.c
 * @brief   SYN6658 语音合成模块实现
 * @note    协议格式: [0xFD] [长度高] [长度低] [命令字] [编码格式] [文本数据...]
 *          长度 = 命令字(1) + 编码格式(1) + 文本长度
 */

#include "syn6658.h"
#include "openmv.h"  /* 用到 OBJ_xxx 定义 */
#include <stdio.h>

extern void delay_ms(__IO uint32_t nTime);

static void USART_SendBytes(USART_TypeDef* USARTx, uint8_t *data, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) {
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTx, data[i]);
    }
}

/**
 * @brief  SYN6658 初始化
 */
void SYN6658_Init(void)
{
    /* 模块上电后需要等待约300ms初始化 */
    delay_ms(300);
}

/**
 * @brief  发送原始数据帧给SYN6658
 * @param  cmd: 命令字
 * @param  data: 数据
 * @param  data_len: 数据长度
 */
static void SYN6658_SendFrame(uint8_t cmd,
                               const uint8_t *data, uint16_t data_len)
{
    uint16_t frame_len = 2 + data_len;  /* 命令字(1) + 编码(1) + 文本 */
    uint8_t header[5];

    /* 如果只是控制命令（无数据），长度=1 */
    if (data == NULL || data_len == 0)
    {
        frame_len = 1;
        header[0] = SYN_HEADER_H;       /* 帧头 0xFD */
        header[1] = 0x00;               /* 长度高字节 */
        header[2] = 0x01;               /* 长度低字节 */
        header[3] = cmd;                /* 命令字 */
        USART_SendBytes(USART2, header, 4);
        return;
    }

    /* 合成播放命令帧 */
    header[0] = SYN_HEADER_H;                      /* 帧头 0xFD */
    header[1] = (uint8_t)((frame_len >> 8) & 0xFF); /* 长度高字节 */
    header[2] = (uint8_t)(frame_len & 0xFF);         /* 长度低字节 */
    header[3] = cmd;                                 /* 命令字 */
    header[4] = SYN_ENCODING_GB2312;                 /* 编码格式 */

    /* 发送帧头+长度+命令+编码 */
    USART_SendBytes(USART2, header, 5);

    /* 发送文本数据 */
    USART_SendBytes(USART2, (uint8_t *)data, data_len);
}

/**
 * @brief  发送文本进行语音合成
 * @note   直接传入中文字符串即可（Keil默认GB2312编码）
 */
void SYN6658_Speak(const char *text)
{
    uint16_t len = (uint16_t)strlen(text);
    SYN6658_SendFrame(SYN_CMD_PLAY, (const uint8_t *)text, len);
}

/**
 * @brief  带音量/语速控制的语音合成
 * @note   通过在文本前面插入控制标记实现
 *         格式: [v10][s5]你好   v=音量(0~16), s=语速(0~10)
 */
void SYN6658_SpeakEx(const char *text,
                     uint8_t volume, uint8_t speed)
{
    char buf[256];

    /* 限幅 */
    if (volume > 16) volume = 16;
    if (speed > 10)  speed = 10;

    /* 在文本前插入控制标记 */
    snprintf(buf, sizeof(buf), "[v%d][s%d]%s", volume, speed, text);

    SYN6658_Speak(buf);
}

/**
 * @brief  停止播报
 */
void SYN6658_Stop(void)
{
    SYN6658_SendFrame(SYN_CMD_STOP, NULL, 0);
}

/**
 * @brief  暂停播报
 */
void SYN6658_Pause(void)
{
    SYN6658_SendFrame(SYN_CMD_PAUSE, NULL, 0);
}

/**
 * @brief  恢复播报
 */
void SYN6658_Resume(void)
{
    SYN6658_SendFrame(SYN_CMD_RESUME, NULL, 0);
}

/**
 * @brief  根据物体ID播报名称
 * @note   比赛现场可快速修改此函数中的文本
 */
void SYN6658_ReportObject(uint8_t object_id)
{
    switch (object_id)
    {
    case OBJ_CIRCLE:
        SYN6658_Speak("检测到圆形");
        break;
    case OBJ_TRIANGLE:
        SYN6658_Speak("检测到三角\xFD"); /* 使用16进制绕过 "形" 字的编码截断问题 */
        break;
    case OBJ_SQUARE:
        SYN6658_Speak("检测到正方\xFD");
        break;
    case OBJ_RED:
        SYN6658_Speak("检测到红色");
        break;
    case OBJ_GREEN:
        SYN6658_Speak("检测到绿色");
        break;
    case OBJ_BLUE:
        SYN6658_Speak("检测到蓝色");
        break;
    default:
        SYN6658_Speak("未识别到目标");
        break;
    }
}

/**
 * @brief  播报巡检点位
 */
void SYN6658_ReportPoint(uint8_t point_num)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "到达目标位置%d", point_num);
    SYN6658_Speak(buf);
}
