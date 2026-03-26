#include "syn6658.h"
#include "openmv.h"
#include <stdio.h>

extern void delay_ms(__IO uint32_t nTime);

#define TTS_STARTUP          "\xBF\xAA\xCA\xBC\xD1\xB2\xBC\xEC"
#define TTS_COLLISION        "\xB7\xA2\xC9\xFA\xC5\xF6\xD7\xB2\xA3\xAC\xBD\xF4\xBC\xB1\xCD\xA3\xD6\xB9"
#define TTS_LIGHTER          "\xB7\xA2\xCF\xD6\xB4\xF2\xBB\xF0\xBB\xFA"
#define TTS_SCISSORS         "\xB7\xA2\xCF\xD6\xBC\xF4\xB5\xB6"
#define TTS_HAMMER           "\xB7\xA2\xCF\xD6\xB4\xB8\xD7\xD3"
#define TTS_UNKNOWN_TARGET   "\xCE\xB4\xCA\xB6\xB1\xF0\xB5\xBD\xC4\xBF\xB1\xEA"
#define TTS_POINT_FMT        "\xB5\xBD\xB4\xEF\xB5\xDA%d\xB8\xF6\xD1\xB2\xBC\xEC\xB5\xE3"

static void USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTx, data[i]);
    }
}

void SYN6658_Init(void)
{
    delay_ms(300);
}

static void SYN6658_SendFrame(uint8_t cmd, const uint8_t *data, uint16_t data_len)
{
    uint16_t frame_len;
    uint8_t header[5];

    if (data == NULL || data_len == 0)
    {
        header[0] = SYN_HEADER_H;
        header[1] = 0x00;
        header[2] = 0x01;
        header[3] = cmd;
        USART_SendBytes(USART2, header, 4);
        return;
    }

    frame_len = (uint16_t)(2 + data_len);

    header[0] = SYN_HEADER_H;
    header[1] = (uint8_t)((frame_len >> 8) & 0xFF);
    header[2] = (uint8_t)(frame_len & 0xFF);
    header[3] = cmd;
    header[4] = SYN_ENCODING_GBK;

    USART_SendBytes(USART2, header, 5);
    USART_SendBytes(USART2, data, data_len);
}

void SYN6658_Speak(const char *text)
{
    uint16_t len = (uint16_t)strlen(text);
    SYN6658_SendFrame(SYN_CMD_PLAY, (const uint8_t *)text, len);
}

void SYN6658_SpeakEx(const char *text, uint8_t volume, uint8_t speed)
{
    char buf[256];

    if (volume > 16) volume = 16;
    if (speed > 10) speed = 10;

    snprintf(buf, sizeof(buf), "[v%d][s%d]%s", volume, speed, text);
    SYN6658_Speak(buf);
}

void SYN6658_Stop(void)
{
    SYN6658_SendFrame(SYN_CMD_STOP, NULL, 0);
}

void SYN6658_Pause(void)
{
    SYN6658_SendFrame(SYN_CMD_PAUSE, NULL, 0);
}

void SYN6658_Resume(void)
{
    SYN6658_SendFrame(SYN_CMD_RESUME, NULL, 0);
}

void SYN6658_ReportObject(uint8_t object_id)
{
    switch (object_id)
    {
    case OBJ_LIGHTER:
        SYN6658_Speak(TTS_LIGHTER);
        break;
    case OBJ_SCISSORS:
        SYN6658_Speak(TTS_SCISSORS);
        break;
    case OBJ_HAMMER:
        SYN6658_Speak(TTS_HAMMER);
        break;
    default:
        SYN6658_Speak(TTS_UNKNOWN_TARGET);
        break;
    }
}

void SYN6658_ReportPoint(uint8_t point_num)
{
    char buf[64];

    snprintf(buf, sizeof(buf), TTS_POINT_FMT, point_num);
    SYN6658_Speak(buf);
}

void SYN6658_ReportStartup(void)
{
    SYN6658_Speak(TTS_STARTUP);
}

void SYN6658_ReportCollision(void)
{
    SYN6658_Speak(TTS_COLLISION);
}

void SYN6658_ReportStation(char point)
{
    char buf[8];
    if (point >= 'A' && point <= 'D') {
        buf[0] = point;
        buf[1] = 0xB5;
        buf[2] = 0xE3;
        buf[3] = 0x00;
        SYN6658_Speak(buf);
    }
}
