/**
 * @file    motor.c
 * @brief   电机驱动实现 —— TB6612FNG PWM控制
 */

#include "motor.h"

/**
 * @brief  电机方向引脚 + STBY使能引脚 GPIO初始化
 */
void Motor_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 使能GPIOC时钟（方向引脚） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* 配置 PC0~PC7 为推挽输出（电机方向） */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                               | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 默认全部拉低（电机停止） */
    GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                        | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

    /* 使能GPIOD时钟（STBY引脚） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* 配置 PD0 为推挽输出（TB6612 STBY使能） */
    GPIO_InitStruct.GPIO_Pin   = MOTOR_STBY_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_STBY_PORT, &GPIO_InitStruct);

    /* 拉高STBY → 使能所有TB6612驱动板 */
    GPIO_SetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
}

/**
 * @brief  使能/禁用电机驱动板
 */
void Motor_Enable(uint8_t enable)
{
    if (enable)
        GPIO_SetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
    else
        GPIO_ResetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
}

/**
 * @brief  设置单个电机速度与方向
 */
void Motor_SetSpeed(uint32_t channel,
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
        GPIO_SetBits(in1_port, in1_pin);
        GPIO_ResetBits(in2_port, in2_pin);
        pwm_val = (uint16_t)speed;
    }
    else if (speed < 0)
    {
        /* 反转: IN1=LOW, IN2=HIGH */
        GPIO_ResetBits(in1_port, in1_pin);
        GPIO_SetBits(in2_port, in2_pin);
        pwm_val = (uint16_t)(-speed);
    }
    else
    {
        /* 停止: IN1=LOW, IN2=LOW */
        GPIO_ResetBits(in1_port, in1_pin);
        GPIO_ResetBits(in2_port, in2_pin);
        pwm_val = 0;
    }

    /* 设置PWM占空比 */
    if (channel == 1) TIM_SetCompare1(TIM1, pwm_val);
    else if (channel == 2) TIM_SetCompare2(TIM1, pwm_val);
    else if (channel == 3) TIM_SetCompare3(TIM1, pwm_val);
    else if (channel == 4) TIM_SetCompare4(TIM1, pwm_val);
}

/**
 * @brief  设置左侧电机（电机1 + 电机3）
 */
void Motor_SetLeft(int16_t speed)
{
    /* 电机1：左前 —— TIM通道1 */
    Motor_SetSpeed(1,
                   MOTOR1_IN1_PORT, MOTOR1_IN1_PIN,
                   MOTOR1_IN2_PORT, MOTOR1_IN2_PIN,
                   speed);

    /* 电机3：左后 —— TIM通道3 */
    Motor_SetSpeed(3,
                   MOTOR3_IN1_PORT, MOTOR3_IN1_PIN,
                   MOTOR3_IN2_PORT, MOTOR3_IN2_PIN,
                   speed);
}

/**
 * @brief  设置右侧电机（电机2 + 电机4）
 */
void Motor_SetRight(int16_t speed)
{
    /* 电机2：右前 —— TIM通道2 */
    Motor_SetSpeed(2,
                   MOTOR2_IN1_PORT, MOTOR2_IN1_PIN,
                   MOTOR2_IN2_PORT, MOTOR2_IN2_PIN,
                   speed);

    /* 电机4：右后 —— TIM通道4 */
    Motor_SetSpeed(4,
                   MOTOR4_IN1_PORT, MOTOR4_IN1_PIN,
                   MOTOR4_IN2_PORT, MOTOR4_IN2_PIN,
                   speed);
}

/**
 * @brief  急停所有电机
 */
void Motor_Stop(void)
{
    Motor_SetLeft(0);
    Motor_SetRight(0);
}
