/**
 * @file    alert.c
 * @brief   声光报警模块实现
 * @note    有源蜂鸣器：高电平=响，低电平=停
 *          LED：高电平=亮，低电平=灭
 */

#include "alert.h"

extern void delay_ms(__IO uint32_t nTime);

/**
 * @brief  声光模块 GPIO 初始化
 */
void Alert_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /* PE1=LED, PE2=蜂鸣器，推挽输出 */
    GPIO_InitStruct.GPIO_Pin   = LED_PIN | BUZZER_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* 默认关闭 */
    GPIO_ResetBits(LED_PORT, LED_PIN);
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

void Alert_LED_On(void)
{
    GPIO_SetBits(LED_PORT, LED_PIN);
}

void Alert_LED_Off(void)
{
    GPIO_ResetBits(LED_PORT, LED_PIN);
}

void Alert_LED_Toggle(void)
{
    GPIO_ToggleBits(LED_PORT, LED_PIN);
}

void Alert_Buzzer_On(void)
{
    GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
}

void Alert_Buzzer_Off(void)
{
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

/**
 * @brief  到达巡检点的声光提示
 * @note   LED亮 + 蜂鸣器响 → 持续duration_ms → 全部关闭
 */
void Alert_Checkpoint(uint16_t duration_ms)
{
    Alert_LED_On();
    Alert_Buzzer_On();
    delay_ms(duration_ms);
    Alert_LED_Off();
    Alert_Buzzer_Off();
}

/**
 * @brief  短促滴一声（200ms）
 */
void Alert_Beep(void)
{
    Alert_Buzzer_On();
    delay_ms(200);
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
        delay_ms(150);
        Alert_LED_Off();
        Alert_Buzzer_Off();
        delay_ms(100);
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
