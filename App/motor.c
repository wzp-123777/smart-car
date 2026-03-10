/**
 * @file    motor.c
 * @brief   电机驱动实现 —— TB6612FNG PWM控制
 */

#include "motor.h"

/**
 * @brief  电机方向引脚GPIO初始化
 * @note   如果已在CubeMX中配置GPIO，可跳过此函数
 */
void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOC时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 配置 PC0~PC7 为推挽输出 */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 默认全部拉低（电机停止） */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                            | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                      GPIO_PIN_RESET);
}

/**
 * @brief  设置单个电机速度与方向
 * @param  speed: 正值=正转, 负值=反转, 0=停止
 */
void Motor_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel,
                    GPIO_TypeDef *in1_port, uint16_t in1_pin,
                    GPIO_TypeDef *in2_port, uint16_t in2_pin,
                    int16_t speed)
{
    uint16_t pwm_val;

    /* 限幅 */
    if (speed > MOTOR_PWM_MAX)  speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0)
    {
        /* 正转: IN1=HIGH, IN2=LOW */
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        pwm_val = (uint16_t)speed;
    }
    else if (speed < 0)
    {
        /* 反转: IN1=LOW, IN2=HIGH */
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        pwm_val = (uint16_t)(-speed);
    }
    else
    {
        /* 停止: IN1=LOW, IN2=LOW */
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        pwm_val = 0;
    }

    /* 设置PWM占空比 */
    __HAL_TIM_SET_COMPARE(htim, channel, pwm_val);
}

/**
 * @brief  设置左侧电机（电机1 + 电机3）
 */
void Motor_SetLeft(TIM_HandleTypeDef *htim, int16_t speed)
{
    /* 电机1：左前 —— TIM通道1 */
    Motor_SetSpeed(htim, TIM_CHANNEL_1,
                   MOTOR1_IN1_PORT, MOTOR1_IN1_PIN,
                   MOTOR1_IN2_PORT, MOTOR1_IN2_PIN,
                   speed);

    /* 电机3：左后 —— TIM通道3 */
    Motor_SetSpeed(htim, TIM_CHANNEL_3,
                   MOTOR3_IN1_PORT, MOTOR3_IN1_PIN,
                   MOTOR3_IN2_PORT, MOTOR3_IN2_PIN,
                   speed);
}

/**
 * @brief  设置右侧电机（电机2 + 电机4）
 */
void Motor_SetRight(TIM_HandleTypeDef *htim, int16_t speed)
{
    /* 电机2：右前 —— TIM通道2 */
    Motor_SetSpeed(htim, TIM_CHANNEL_2,
                   MOTOR2_IN1_PORT, MOTOR2_IN1_PIN,
                   MOTOR2_IN2_PORT, MOTOR2_IN2_PIN,
                   speed);

    /* 电机4：右后 —— TIM通道4 */
    Motor_SetSpeed(htim, TIM_CHANNEL_4,
                   MOTOR4_IN1_PORT, MOTOR4_IN1_PIN,
                   MOTOR4_IN2_PORT, MOTOR4_IN2_PIN,
                   speed);
}

/**
 * @brief  急停所有电机
 */
void Motor_Stop(TIM_HandleTypeDef *htim)
{
    Motor_SetLeft(htim, 0);
    Motor_SetRight(htim, 0);
}
