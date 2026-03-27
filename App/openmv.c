#include "openmv.h"

volatile OpenMV_DataTypeDef g_openmv_data = {0};
uint8_t g_openmv_rx_buf[OPENMV_RX_BUF_SIZE] = {0};
uint8_t g_openmv_rx_index = 0;

static char s_rx_str[128] = {0};
static uint8_t s_rx_str_idx = 0;

void OpenMV_Init(void)
{
    s_rx_str_idx = 0;
    g_openmv_data.is_valid = 0;
    g_openmv_data.is_new = 0;
}

void OpenMV_SendCmd(uint8_t cmd) { (void)cmd; }

void OpenMV_ParseByte(uint8_t byte)
{
    if (byte == '\n' || byte == '\r')
    {
        if (s_rx_str_idx > 0)
        {
            s_rx_str[s_rx_str_idx] = '\0';

            if (strstr(s_rx_str, "Lighter") != NULL || strstr(s_rx_str, "lighter") != NULL) {
                g_openmv_data.object_id = OBJ_LIGHTER;
                g_openmv_data.is_valid = 1;
                g_openmv_data.is_new = 1;
            } else if (strstr(s_rx_str, "Scissors") != NULL || strstr(s_rx_str, "scissors") != NULL) {
                g_openmv_data.object_id = OBJ_SCISSORS;
                g_openmv_data.is_valid = 1;
                g_openmv_data.is_new = 1;
            } else if (strstr(s_rx_str, "Hammer") != NULL || strstr(s_rx_str, "hammer") != NULL) {
                g_openmv_data.object_id = OBJ_HAMMER;
                g_openmv_data.is_valid = 1;
                g_openmv_data.is_new = 1;
            }
            s_rx_str_idx = 0;
        }
    }
    else
    {
        if (byte >= 0x80) {
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
