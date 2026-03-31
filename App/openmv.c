#include "openmv.h"

volatile OpenMV_DataTypeDef g_openmv_data = {0};
uint8_t g_openmv_rx_buf[OPENMV_RX_BUF_SIZE] = {0};
uint8_t g_openmv_rx_index = 0;
volatile uint32_t g_openmv_rx_byte_count = 0;

static char s_rx_str[128] = {0};
static uint8_t s_rx_str_idx = 0;

static void OpenMV_SetObject(uint8_t object_id)
{
    g_openmv_data.object_id = object_id;
    g_openmv_data.is_valid = 1;
    g_openmv_data.is_new = 1;
}

static void OpenMV_ParseTextLine(void)
{
    s_rx_str[s_rx_str_idx] = '\0';

    if (strstr(s_rx_str, "Lighter") != NULL || strstr(s_rx_str, "lighter") != NULL)
    {
        OpenMV_SetObject(OBJ_LIGHTER);
    }
    else if (strstr(s_rx_str, "Scissors") != NULL || strstr(s_rx_str, "scissors") != NULL)
    {
        OpenMV_SetObject(OBJ_SCISSORS);
    }
    else if (strstr(s_rx_str, "Hammer") != NULL || strstr(s_rx_str, "hammer") != NULL)
    {
        OpenMV_SetObject(OBJ_HAMMER);
    }
}

static void OpenMV_ParseBinaryByte(uint8_t byte)
{
    if (g_openmv_rx_index == 0U)
    {
        if ((byte == OPENMV_RX_HEADER) || (byte == 0xA5U))
        {
            g_openmv_rx_buf[g_openmv_rx_index++] = byte;
        }
        return;
    }

    if (g_openmv_rx_index < OPENMV_RX_BUF_SIZE)
    {
        g_openmv_rx_buf[g_openmv_rx_index++] = byte;
    }
    else
    {
        g_openmv_rx_index = 0U;
        return;
    }

    if ((g_openmv_rx_buf[0] == 0xA5U) && (g_openmv_rx_index >= 3U))
    {
        if (g_openmv_rx_buf[2] == 0x5AU)
        {
            uint8_t object_id = g_openmv_rx_buf[1];

            if ((object_id == OBJ_LIGHTER) ||
                (object_id == OBJ_SCISSORS) ||
                (object_id == OBJ_HAMMER))
            {
                OpenMV_SetObject(object_id);
            }
        }

        g_openmv_rx_index = 0U;
        return;
    }

    if (g_openmv_rx_index >= 8U)
    {
        if ((g_openmv_rx_buf[0] == OPENMV_RX_HEADER) &&
            (g_openmv_rx_buf[7] == OPENMV_RX_TAIL))
        {
            uint8_t object_id = g_openmv_rx_buf[1];

            if ((object_id == OBJ_LIGHTER) ||
                (object_id == OBJ_SCISSORS) ||
                (object_id == OBJ_HAMMER))
            {
                OpenMV_SetObject(object_id);
            }
        }

        g_openmv_rx_index = 0U;
    }
}

void OpenMV_Init(void)
{
    s_rx_str_idx = 0;
    g_openmv_rx_index = 0;
    g_openmv_rx_byte_count = 0;
    g_openmv_data.is_valid = 0;
    g_openmv_data.is_new = 0;
}

void OpenMV_SendCmd(uint8_t cmd) { (void)cmd; }

void OpenMV_ParseByte(uint8_t byte)
{
    g_openmv_rx_byte_count++;
    OpenMV_ParseBinaryByte(byte);

    if (byte == '\n' || byte == '\r')
    {
        if (s_rx_str_idx > 0)
        {
            OpenMV_ParseTextLine();
        }
        s_rx_str_idx = 0;
    }
    else
    {
        if (byte >= 0x80)
        {
            s_rx_str_idx = 0;
            return;
        }

        if (s_rx_str_idx < sizeof(s_rx_str) - 1)
        {
            s_rx_str[s_rx_str_idx++] = (char)byte;
        }
    }
}

OpenMV_DataTypeDef OpenMV_GetResult(void)
{
    return g_openmv_data;
}

uint8_t OpenMV_HasNewData(void)
{
    return g_openmv_data.is_new;
}

void OpenMV_ClearNewFlag(void)
{
    g_openmv_data.is_new = 0;
}

uint32_t OpenMV_GetRxByteCount(void)
{
    return g_openmv_rx_byte_count;
}
