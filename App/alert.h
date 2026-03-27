/**
 * @file    alert.h
 * @brief   声光报警模块 —— LED
 * @note    到达巡检点时亮灯
 *
 * 【接线】
 *  LED:    PE1 → LED正极（串联限流电阻220Ω到3.3V，或直接接LED模块的信号脚）    
 */
#ifndef __ALERT_H
#define __ALERT_H

#include "stm32f4xx.h"

/* ==================== 引脚定义（按实际接线修改）==================== */       
#define LED_PORT        GPIOE
#define LED_PIN         GPIO_Pin_1

/* ==================== 函数声明 ==================== */

/**
 * @brief  LED GPIO 初始化
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
 * @brief  到达巡检点时的光提示（亮灯一段时间）
 * @param  duration_ms: 持续时间（毫秒），如 500 = 亮0.5秒
 * @note   阻塞式，会占用CPU等待。适合在停车状态下调用
 */
void Alert_Checkpoint(uint16_t duration_ms);

/**
 * @brief  错误报警（快速闪烁）
 * @param  count: 闪烁次数
 */
void Alert_Error(uint8_t count);

/**
 * @brief  全部关闭（LED灭）
 */
void Alert_AllOff(void);

#endif /* __ALERT_H */
