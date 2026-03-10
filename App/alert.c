/**
 * @file    alert.c
 * @brief   声光报警模块实现
 * @note    有源蜂鸣器：高电平=响，低电平=停
 *          LED：高电平=亮，低电平=灭
 */

#include "alert.h"

/**
 * @brief  声光模块 GPIO 初始化
 */
void Alert_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* PE1=LED, PE2=蜂鸣器，推挽输出 */
    GPIO_InitStruct.Pin   = LED_PIN | BUZZER_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* 默认关闭 */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void Alert_LED_On(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void Alert_LED_Off(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void Alert_LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

void Alert_Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void Alert_Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  到达巡检点的声光提示
 * @note   LED亮 + 蜂鸣器响 → 持续duration_ms → 全部关闭
 */
void Alert_Checkpoint(uint16_t duration_ms)
{
    Alert_LED_On();
    Alert_Buzzer_On();
    HAL_Delay(duration_ms);
    Alert_LED_Off();
    Alert_Buzzer_Off();
}

/**
 * @brief  短促滴一声（200ms）
 */
void Alert_Beep(void)
{
    Alert_Buzzer_On();
    HAL_Delay(200);
    Alert_Buzzer_Off();
}

/**
 * @brief  错误报警（快速闪烁+蜂鸣）
 */
void Alert_Error(uint8_t count)
{
    uint8_t i;
    for (i = 0; i < count; i++)
    {
        Alert_LED_On();
        Alert_Buzzer_On();
        HAL_Delay(150);
        Alert_LED_Off();
        Alert_Buzzer_Off();
        HAL_Delay(100);
    }
}

/**
 * @brief  全部关闭
 */
void Alert_AllOff(void)
{
    Alert_LED_Off();
    Alert_Buzzer_Off();
}
