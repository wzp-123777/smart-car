/**
 * @file    alert.h
 * @brief   声光报警模块 —— LED + 蜂鸣器
 * @note    到达巡检点时亮灯+鸣笛
 * 
 * 【接线】
 *  LED:    PE1 → LED正极（串联限流电阻220Ω到3.3V，或直接接LED模块的信号脚）
 *  蜂鸣器: PE2 → 有源蜂鸣器模块 信号脚（高电平响）
 */
#ifndef __ALERT_H
#define __ALERT_H

#include "stm32f4xx_hal.h"

/* ==================== 引脚定义（按实际接线修改）==================== */
#define LED_PORT        GPIOE
#define LED_PIN         GPIO_PIN_1

#define BUZZER_PORT     GPIOE
#define BUZZER_PIN      GPIO_PIN_2

/* ==================== 函数声明 ==================== */

/**
 * @brief  LED + 蜂鸣器 GPIO 初始化
 */
void Alert_Init(void);

/**
 * @brief  LED 开
 */
void Alert_LED_On(void);

/**
 * @brief  LED 关
 */
void Alert_LED_Off(void);

/**
 * @brief  LED 翻转
 */
void Alert_LED_Toggle(void);

/**
 * @brief  蜂鸣器 开
 */
void Alert_Buzzer_On(void);

/**
 * @brief  蜂鸣器 关
 */
void Alert_Buzzer_Off(void);

/**
 * @brief  到达巡检点时的声光提示（亮灯 + 鸣笛一段时间）
 * @param  duration_ms: 持续时间（毫秒），如 500 = 响0.5秒
 * @note   阻塞式，会占用CPU等待。适合在停车状态下调用
 */
void Alert_Checkpoint(uint16_t duration_ms);

/**
 * @brief  短促提示音（滴一下，200ms）
 */
void Alert_Beep(void);

/**
 * @brief  错误报警（快速闪烁+连续蜂鸣）
 * @param  count: 闪烁/鸣叫次数
 */
void Alert_Error(uint8_t count);

/**
 * @brief  全部关闭（LED灭 + 蜂鸣器停）
 */
void Alert_AllOff(void);

#endif /* __ALERT_H */
