/**
 * @file    motor.h
 * @brief   电机驱动模块 —— 4路 TB6612FNG 控制
 * @note    每个TB6612控制一个N20电机，通过PWM+方向引脚控制
 * 
 * 【引脚分配建议】（根据实际接线修改宏定义即可）
 *  电机1(左前): TIM1_CH1(PA8)  + AIN1(PC0) + AIN2(PC1)
 *  电机2(右前): TIM1_CH2(PA9)  + BIN1(PC2) + BIN2(PC3)
 *  电机3(左后): TIM1_CH3(PA10) + AIN1(PC4) + AIN2(PC5)
 *  电机4(右后): TIM1_CH4(PA11) + BIN1(PC6) + BIN2(PC7)
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

/* PWM最大值（与定时器ARR对应，默认1000） */
#define MOTOR_PWM_MAX       1000

/* ===================== 函数声明 ===================== */

/**
 * @brief  电机GPIO初始化（方向引脚）
 * @note   PWM定时器需要在CubeMX中配置或手动初始化
 */
void Motor_GPIO_Init(void);

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
