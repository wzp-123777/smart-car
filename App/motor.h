/**
 * @file    motor.h
 * @brief   电机驱动模块 —— 4路 TB6612FNG 控制
 * @note    每个TB6612控制一个N20电机，通过PWM+方向引脚控制
 * 
 * 【完整接线】每个 TB6612FNG 模块有这些引脚：
 *
 *  STM32 引脚          TB6612 引脚        说明
 *  ─────────────────────────────────────────────
 *  PA8  (TIM1_CH1)  →  PWMA             电机1速度（左前）
 *  PC0  (GPIO)      →  AIN1             电机1方向
 *  PC1  (GPIO)      →  AIN2             电机1方向
 *  PD0  (GPIO)      →  STBY             使能（4个模块STBY并联）
 *                       AO1/AO2  →  电机1 两根线
 *                       VM       →  电机电源 6~12V
 *                       VCC      →  3.3V
 *                       GND      →  GND
 *
 *  PA9  (TIM1_CH2)  →  PWMA             电机2速度（右前）
 *  PC2  (GPIO)      →  AIN1             电机2方向
 *  PC3  (GPIO)      →  AIN2             电机2方向
 *
 *  PA10 (TIM1_CH3)  →  PWMA             电机3速度（左后）
 *  PC4  (GPIO)      →  AIN1             电机3方向
 *  PC5  (GPIO)      →  AIN2             电机3方向
 *
 *  PA11 (TIM1_CH4)  →  PWMA             电机4速度（右后）
 *  PC6  (GPIO)      →  AIN1             电机4方向
 *  PC7  (GPIO)      →  AIN2             电机4方向
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"

/* ===================== 引脚定义（按实际接线修改）===================== */

/* --- 电机1：左前 --- */
#define MOTOR1_IN1_PORT     GPIOC
#define MOTOR1_IN1_PIN      GPIO_PIN_0
#define MOTOR1_IN2_PORT     GPIOC
#define MOTOR1_IN2_PIN      GPIO_PIN_1

/* --- 电机2：右前 --- */
#define MOTOR2_IN1_PORT     GPIOC
#define MOTOR2_IN1_PIN      GPIO_PIN_2
#define MOTOR2_IN2_PORT     GPIOC
#define MOTOR2_IN2_PIN      GPIO_PIN_3

/* --- 电机3：左后 --- */
#define MOTOR3_IN1_PORT     GPIOC
#define MOTOR3_IN1_PIN      GPIO_PIN_4
#define MOTOR3_IN2_PORT     GPIOC
#define MOTOR3_IN2_PIN      GPIO_PIN_5

/* --- 电机4：右后 --- */
#define MOTOR4_IN1_PORT     GPIOC
#define MOTOR4_IN1_PIN      GPIO_PIN_6
#define MOTOR4_IN2_PORT     GPIOC
#define MOTOR4_IN2_PIN      GPIO_PIN_7

/* --- TB6612 STBY 使能引脚（4个模块的STBY并联到同一个GPIO） --- */
#define MOTOR_STBY_PORT     GPIOD
#define MOTOR_STBY_PIN      GPIO_PIN_0

/* PWM最大值（与定时器ARR对应，默认1000） */
#define MOTOR_PWM_MAX       1000

/* ===================== 函数声明 ===================== */

/**
 * @brief  电机GPIO初始化（方向引脚 + STBY使能引脚）
 * @note   初始化后自动拉高STBY使能驱动板
 */
void Motor_GPIO_Init(void);

/**
 * @brief  使能/禁用电机驱动板
 * @param  enable: 1=使能（STBY=HIGH）, 0=禁用（STBY=LOW，硬件级停车）
 */
void Motor_Enable(uint8_t enable);

/**
 * @brief  设置单个电机速度
 * @param  htim: PWM定时器句柄
 * @param  channel: 定时器通道 (TIM_CHANNEL_1 ~ 4)
 * @param  motor_in1_port/pin: 方向引脚1
 * @param  motor_in2_port/pin: 方向引脚2
 * @param  speed: 速度值（-1000 ~ +1000，正=正转，负=反转）
 */
void Motor_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel,
                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                    GPIO_TypeDef *in2_port, uint16_t in2_pin,
                    int16_t speed);

/**
 * @brief  设置左侧两个电机（电机1+电机3）
 * @param  speed: -1000 ~ +1000
 */
void Motor_SetLeft(TIM_HandleTypeDef *htim, int16_t speed);

/**
 * @brief  设置右侧两个电机（电机2+电机4）
 * @param  speed: -1000 ~ +1000
 */
void Motor_SetRight(TIM_HandleTypeDef *htim, int16_t speed);

/**
 * @brief  急停：所有电机停止
 */
void Motor_Stop(TIM_HandleTypeDef *htim);

#endif /* __MOTOR_H */
