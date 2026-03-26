#pragma diag_suppress 177
/**
* @file    main.c
* @brief   智能巡护小车主程序
* @author  2026开发团队
* @note
*   主控: STM32F407ZGT6 (标准库 SPL)
*   视觉: OpenMV H7 Plus (UART1, 115200, PB6=TX, PB7=RX)
*   语音: SYN6658 (UART2, 9600, PA2=TX, PA3=RX)
*   动力: 4路 N20 + 4路 TB6612FNG (TIM1 PWM, PA8~PA11)
*   编码器: 左前=TIM2(PA0,PA1), 右前=TIM3(PA6,PA7)
*   循迹: 5路红外 (PD1~PD5)
*   姿态: MPU6050 (USART3, 115200, PB10=TX, PB11=RX)
*   报警: 声光模块 (Alert_Init)
*   PID定时器: TIM6 (10ms触发)
*
* 编程规范与说明:
*   当前工程不使用 CubeMX 和 HAL 库。
*   1. 采用标准外设库 CMSIS 和 STM32F4xx_StdPeriph_Driver。
*   2. 用户代码在 App 目录下。
*   3. 需定义 USE_STDPERIPH_DRIVER 宏。
*   4. 避免使用 CubeMX 重新生成代码。
*/
#include "stm32f4xx.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "infrared.h"
#include "openmv.h"
#include "syn6658.h"
#include "mpu6050.h"
#include "state_machine.h"
#include "alert.h"
#include <stdio.h>
/* ================================================================
*                     全局变量定义
* ================================================================ */
/* PID 左右轮控制 */
PID_TypeDef g_pid_left;
PID_TypeDef g_pid_right;
/* 转向 PID 控制 */
PID_TypeDef g_pid_turn;
/* 传感器数据 */
IR_DataTypeDef g_ir_data;
/* MPU6050 数据 */
MPU6050_DataTypeDef g_mpu_data;
/* 状态机 */
StateMachine_TypeDef g_state_machine;
/* 编码器读数 */
volatile int16_t g_encoder_left  = 0;
volatile int16_t g_encoder_right = 0;
/* 基础速度 / 10ms 周期 */
int16_t g_base_speed = 15;
/* PID 计算后的 PWM 值 */
volatile float g_motor_left_pwm  = 0;
volatile float g_motor_right_pwm = 0;
/* 10ms 中断标志 */
volatile uint8_t g_flag_10ms = 0;

static int16_t g_line_debug_left_pwm = 0;
static int16_t g_line_debug_right_pwm = 0;
static uint8_t g_line_debug_left_score = 0;
static uint8_t g_line_debug_right_score = 0;
static const char *g_line_debug_mode = "INIT";

#define LINE_DEBUG_INTERVAL_MS     150U
#define LINE_FOLLOW_LOOP_DELAY_MS    5U
#define LINE_FOLLOW_TARGET_STRAIGHT    35.0f   // 提速前为 20.0
#define LINE_FOLLOW_TARGET_FINE        30.0f   // 提速前为 18.0
#define LINE_FOLLOW_TARGET_STRONG      25.0f   // 提速前为 14.0
#define LINE_FOLLOW_TARGET_LOST         0.0f
#define LINE_FOLLOW_TARGET_SEARCH      20.0f   // 提速前为 12.0
#define LINE_FOLLOW_FINE_ADJUST        14.0f   // 放大转弯差速
#define LINE_FOLLOW_STRONG_ADJUST      25.0f   // 放大转弯差速
#define LINE_FOLLOW_SEARCH_ADJUST      40.0f   // 放大转弯差速
#define LINE_FOLLOW_GYRO_DAMP           0.05f
#define LINE_FOLLOW_GYRO_DAMP_MAX       4.0f
#define LINE_FOLLOW_MIN_INNER_SPEED    -45.0f  // 允许内轮反转，增大反转幅度防抱死
#define LINE_FOLLOW_GYRO_CAL_SAMPLES   12U
#define LINE_FOLLOW_GYRO_CAL_DELAY_MS   5U
/* ================================================================
*                     系统滴答定时器函数 SysTick 相关
* ================================================================ */
volatile uint32_t g_sys_tick = 0;
uint32_t HAL_GetTick(void)
{
return g_sys_tick;
}
/* SysTick_Handler 在 stm32f4xx_it.c 中定义
* 并在该函数中累加 g_sys_tick 变量
*/
// void SysTick_Handler(void)
// {
//     g_sys_tick++;
// }
/* ms 级延时函数，基于 SysTick 时基 */
void delay_ms(__IO uint32_t nTime)
{
uint32_t start_time = HAL_GetTick();
while ((HAL_GetTick() - start_time) < nTime);
}

static void Debug_USART_SendString(USART_TypeDef *USARTx, const char *str)
{
while (*str != '\0')
{
while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
USART_SendData(USARTx, (uint8_t)(*str));
str++;
}
}

static void Debug_LogIR(CarEvent_TypeDef event)
{
char buf[168];
static uint32_t last_debug_tick = 0;
uint32_t now;

now = HAL_GetTick();
if ((now - last_debug_tick) < LINE_DEBUG_INTERVAL_MS)
{
return;
}
last_debug_tick = now;

snprintf(buf,
         sizeof(buf),
         "IR bits=%u%u%u%u%u raw=0x%02X pos=%d aw=%u ab=%u ls=%u rs=%u lp=%d rp=%d el=%d er=%d mode=%s st=%s ev=%d\r\n",
         g_ir_data.sensor[0],
         g_ir_data.sensor[1],
         g_ir_data.sensor[2],
         g_ir_data.sensor[3],
         g_ir_data.sensor[4],
         g_ir_data.raw_byte,
         g_ir_data.position,
         g_ir_data.all_white,
         g_ir_data.all_black,
         g_line_debug_left_score,
         g_line_debug_right_score,
         g_line_debug_left_pwm,
         g_line_debug_right_pwm,
         g_encoder_left,
         g_encoder_right,
         g_line_debug_mode,
         SM_GetStateName(g_state_machine.current_state),
         (int)event);
Debug_USART_SendString(USART3, buf);
}

static void Debug_LogText(const char *text)
{
Debug_USART_SendString(USART3, text);
}

static float g_mpu_gyro_z_bias = 0.0f;

static float LineFollow_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }
    return value;
}

static float LineFollow_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static void LineFollow_CalibrateGyroBias(void)
{
    MPU6050_DataTypeDef sample;
    float gyro_sum = 0.0f;
    uint8_t i;

    for (i = 0; i < LINE_FOLLOW_GYRO_CAL_SAMPLES; i++)
    {
        MPU6050_ReadAll(&sample);
        gyro_sum += sample.gyro_z_dps;
        delay_ms(LINE_FOLLOW_GYRO_CAL_DELAY_MS);
    }

    g_mpu_gyro_z_bias = gyro_sum / (float)LINE_FOLLOW_GYRO_CAL_SAMPLES;
}

void LineFollow_RunByRawIR(void)
{
    float base_speed = LINE_FOLLOW_TARGET_STRAIGHT;
    float steer_strength = 0.0f;
    float gyro_damp = 0.0f;
    float gyro_rate = g_mpu_data.gyro_z_dps - g_mpu_gyro_z_bias;
    float speed_left = LINE_FOLLOW_TARGET_STRAIGHT;
    float speed_right = LINE_FOLLOW_TARGET_STRAIGHT;
    uint8_t left_edge_on = g_ir_data.sensor[0];
    uint8_t left_near_on = g_ir_data.sensor[1];
    uint8_t center_on = g_ir_data.sensor[2];
    uint8_t right_near_on = g_ir_data.sensor[3];
    uint8_t right_edge_on = g_ir_data.sensor[4];
    uint8_t left_score = (left_edge_on ? 2U : 0U) + (left_near_on ? 1U : 0U);
    uint8_t right_score = (right_edge_on ? 2U : 0U) + (right_near_on ? 1U : 0U);
    int8_t turn_dir = 0;

    g_line_debug_left_score = left_score;
    g_line_debug_right_score = right_score;

    if (g_ir_data.all_white != 0U)
    {
        base_speed = LINE_FOLLOW_TARGET_LOST;
        steer_strength = 0.0f;
        g_line_debug_mode = "LOST_0";
    }
    else if (left_score > right_score)
    {
        turn_dir = -1;

        if (left_edge_on != 0U)
        {
            base_speed = center_on ? LINE_FOLLOW_TARGET_STRONG : LINE_FOLLOW_TARGET_SEARCH;
            steer_strength = center_on ? LINE_FOLLOW_STRONG_ADJUST : LINE_FOLLOW_SEARCH_ADJUST;
            g_line_debug_mode = center_on ? "EDGE_L" : "HARD_L";
        }
        else
        {
            base_speed = LINE_FOLLOW_TARGET_FINE;
            steer_strength = LINE_FOLLOW_FINE_ADJUST;
            g_line_debug_mode = "FINE_L";
        }
    }
    else if (right_score > left_score)
    {
        turn_dir = 1;

        if (right_edge_on != 0U)
        {
            base_speed = center_on ? LINE_FOLLOW_TARGET_STRONG : LINE_FOLLOW_TARGET_SEARCH;
            steer_strength = center_on ? LINE_FOLLOW_STRONG_ADJUST : LINE_FOLLOW_SEARCH_ADJUST;
            g_line_debug_mode = center_on ? "EDGE_R" : "HARD_R";
        }
        else
        {
            base_speed = LINE_FOLLOW_TARGET_FINE;
            steer_strength = LINE_FOLLOW_FINE_ADJUST;
            g_line_debug_mode = "FINE_R";
        }
    }
    else
    {
        if ((center_on != 0U) || (g_ir_data.all_black != 0U))
        {
            base_speed = LINE_FOLLOW_TARGET_STRAIGHT;
            g_line_debug_mode = (g_ir_data.all_black != 0U) ? "BLACK" : "CENTER";
        }
        else
        {
            base_speed = LINE_FOLLOW_TARGET_FINE;
            g_line_debug_mode = "BAL";
        }
    }

    if (steer_strength > 0.0f)
    {
        gyro_damp = LineFollow_AbsFloat(gyro_rate) * LINE_FOLLOW_GYRO_DAMP;
        gyro_damp = LineFollow_ClampFloat(gyro_damp, 0.0f, LINE_FOLLOW_GYRO_DAMP_MAX);

        if ((left_edge_on != 0U) || (right_edge_on != 0U))
        {
            steer_strength = LineFollow_ClampFloat(steer_strength - gyro_damp, 2.0f, LINE_FOLLOW_SEARCH_ADJUST);
        }
        else
        {
            steer_strength = LineFollow_ClampFloat(steer_strength - gyro_damp, 0.8f, LINE_FOLLOW_FINE_ADJUST);
        }
    }

    speed_left = base_speed;
    speed_right = base_speed;

    if (turn_dir < 0)
    {
        /* 当前实车方向：左侧红外命中时，由右侧作为内轮减速修正。 */
        speed_right = LineFollow_ClampFloat(base_speed - steer_strength,
                                            LINE_FOLLOW_MIN_INNER_SPEED,
                                            LINE_FOLLOW_TARGET_STRAIGHT);
    }
    else if (turn_dir > 0)
    {
        /* 当前实车方向：右侧红外命中时，由左侧作为内轮减速修正。 */
        speed_left = LineFollow_ClampFloat(base_speed - steer_strength,
                                           LINE_FOLLOW_MIN_INNER_SPEED,
                                           LINE_FOLLOW_TARGET_STRAIGHT);
    }

    g_pid_left.target = speed_left;
    g_pid_right.target = speed_right;
    g_line_debug_left_pwm = (int16_t)speed_left;
    g_line_debug_right_pwm = (int16_t)speed_right;
}
/* ================================================================
*                     ???????????? (??????)
* ================================================================ */
/**
* @brief  TIM1 PWM????? ???? ??????? (168MHz / 168 / 1000 = 1kHz)
*         PA8~PA11 (CH1~CH4)
*/
static void MX_TIM1_PWM_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
TIM_TimeBaseStructure.TIM_Prescaler = 168 - 1;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse = 0;
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OC1Init(TIM1, &TIM_OCInitStructure);
TIM_OC2Init(TIM1, &TIM_OCInitStructure);
TIM_OC3Init(TIM1, &TIM_OCInitStructure);
TIM_OC4Init(TIM1, &TIM_OCInitStructure);
TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
TIM_ARRPreloadConfig(TIM1, ENABLE);
TIM_Cmd(TIM1, ENABLE);
TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
/**
* @brief  TIM2 ???????? ???? ????
*         PA0(CH1), PA1(CH2)
*/
static void MX_TIM2_Encoder_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
TIM_SetCounter(TIM2, 0);
TIM_Cmd(TIM2, ENABLE);
}
/**
* @brief  TIM3 ???????? ???? ????
*         PA6(CH1), PA7(CH2)
*/
static void MX_TIM3_Encoder_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
TIM_SetCounter(TIM3, 0);
TIM_Cmd(TIM3, ENABLE);
}
/**
* @brief  TIM6 ????????? ???? 10ms PID????????
*         84MHz(APB1) / 840 / 1000 = 100Hz (10ms)
*/
static void MX_TIM6_Init(void)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
NVIC_InitTypeDef NVIC_InitStructure;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
TIM_TimeBaseStructure.TIM_Prescaler = 840 - 1;
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
TIM_Cmd(TIM6, ENABLE);
}
/**
* @brief  UART1 ????? ???? OpenMV_TX/RX (PB6/PB7)
*/
static void MX_USART1_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(GPIOB, &GPIO_InitStructure);
USART_InitStructure.USART_BaudRate = 115200;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART_Init(USART1, &USART_InitStructure);
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
USART_Cmd(USART1, ENABLE);
}
/**
* @brief  UART2 ????? ???? SYN6658 ???? (PA2/PA3)
*/
static void MX_USART2_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
USART_InitStructure.USART_BaudRate = 9600;
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART_Init(USART2, &USART_InitStructure);
USART_Cmd(USART2, ENABLE);
}
/**
* @brief  USART3 ????? ???? MPU6050 ????? (PB10=TX, PB11=RX)
*/
static void MX_USART3_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(GPIOB, &GPIO_InitStructure);
USART_InitStructure.USART_BaudRate = 115200; // JY61????????????115200????9600
USART_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_InitStructure.USART_StopBits = USART_StopBits_1;
USART_InitStructure.USART_Parity = USART_Parity_No;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART_Init(USART3, &USART_InitStructure);
NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
USART_Cmd(USART3, ENABLE);
}
/**
* @brief  I2C1 ????? ???? MPU6050 (PB8=SCL, PB9=SDA)
*/
/* ================================================================
*                     ????????????? (???????)
* ================================================================ */
/**
* @brief  USART1 ????????? ???? OpenMV ???????
*/
void USART1_IRQHandler(void)
{
if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
{
uint8_t RxData = USART_ReceiveData(USART1);
OpenMV_ParseByte(RxData); /* ??????????????????????????????? */
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}    /* 重点修复：清除可能由OpenMV上电连续发送导致的溢出中断(ORE)死锁 */
if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
{
USART_ReceiveData(USART1); /* 读数据寄存器清除ORE */
}}
/**
* @brief  TIM6 ????????? ???? 10ms PID?????????
*/
void TIM6_DAC_IRQHandler(void)
{
    float pwm_left = 0.0f;
    float pwm_right = 0.0f;

    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        
        /* ===== 1. 获取编码器反馈值 ===== */
        g_encoder_left  = Encoder_Read_TIM2(); 
        g_encoder_right = Encoder_Read_TIM3();
        
        /* ===== 2. MPU6050 姿态结算 ===== */
        MPU6050_UpdateYaw(&g_mpu_data, 0.01f);

        /* ===== 4. 内环：PID速度闭环执行 ===== */
        if (g_pid_left.target == 0.0f)
        {
            PID_Reset(&g_pid_left);
            pwm_left = 0.0f;
        }
        else
        {
            pwm_left = PID_Incremental(&g_pid_left, g_pid_left.target, (float)g_encoder_left);
        }

        if (g_pid_right.target == 0.0f)
        {
            PID_Reset(&g_pid_right);
            pwm_right = 0.0f;
        }
        else
        {
            pwm_right = PID_Incremental(&g_pid_right, g_pid_right.target, (float)g_encoder_right);
        }
        
        /* 驱动电机输出PWM */
        Motor_SetLeft((int16_t)pwm_left);
        Motor_SetRight((int16_t)pwm_right);

        /* 发送事件标志给主循环 */
        g_flag_10ms = 1;
    }
}

/* ================================================================
*                     ????????
* ================================================================ */
/**
* @brief  ???????????????????
*/
static CarEvent_TypeDef DetectEvent(void)
{
    static uint32_t last_speak_time = 0;
    static uint8_t  last_object_id = 0;

    if (OpenMV_HasNewData())
    {
        OpenMV_DataTypeDef result = OpenMV_GetResult();
        OpenMV_ClearNewFlag();

        if (result.is_valid && result.object_id != 0)
        {
            uint32_t current_time = HAL_GetTick();
            
            /* 如果是新物体，或者距离上次播报同一个物体已经超过3000ms，则播报 */
            if ((result.object_id != last_object_id) || (current_time - last_speak_time > 3000))
            {
                SYN6658_ReportObject(result.object_id);
                last_object_id = result.object_id;
                last_speak_time = current_time;
            }
        }
    }
    return EVENT_NONE;
}
/* ================================================================
*                     ????????
* ================================================================ */
/**
* @brief  ???????????????PA15????????????
*/
static void StartButton_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
/* PA15 ????? JTAG ?? JTDI ???????????????????????????????????????? */
// ????????????????????
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
// ??????F1????????JTAG??F4????????????????????????????????????????
// ??????????????????????????????????????????????????????
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/**
* @brief  ??????? (??????????)
*/
static uint8_t StartButton_IsPressed(void)
{
if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET)
{
delay_ms(20);
if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET)
{
while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET);
return 1;
}
}
return 0;
}
/* ================================================================
*                         ??????
* ================================================================ */

/* ==================== 语音播报 GBK 编码 ==================== */
#define TTS_PLEASE_SELECT     "\xC7\xEB\xD1\xA1\xD4\xF1\xC8\xCE\xCE\xF1" // "请选择任务"
#define TTS_NO_TASK_SELECTED  "\xCE\xB4\xD1\xA1\xD4\xF1\xC8\xCE\xCE\xF1" // "未选择任务"
#define TTS_START_EXEC        "\xBF\xAA\xCA\xBC\xD4\xCB\xD0\xD0"         // "开始执行"

// "任务1" ... "任务4"的GBK
char* TASK_NAMES[5] = {
    "",
    "\xC8\xCE\xCE\xF1\x31",
    "\xC8\xCE\xCE\xF1\x32",
    "\xC8\xCE\xCE\xF1\x33",
    "\xC8\xCE\xCE\xF1\x34"
};

// "进入任务1" ~ "进入任务4"
char* ENTER_TASK_NAMES[5] = {
    "",
    "\xBD\xF8\xC8\xEB\xC8\xCE\xCE\xF1\x31",
    "\xBD\xF8\xC8\xEB\xC8\xCE\xCE\xF1\x32",
    "\xBD\xF8\xC8\xEB\xC8\xCE\xCE\xF1\x33",
    "\xBD\xF8\xC8\xEB\xC8\xCE\xCE\xF1\x34"
};

/* ==================== 系统调度状态 ==================== */
typedef enum {
    SYS_STATE_TASK_SELECT = 0,
    SYS_STATE_TASK_RUNNING
} SystemState_t;

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,
    KEY_EVENT_LONG
} KeyEvent_t;

SystemState_t g_sys_state = SYS_STATE_TASK_SELECT;
uint8_t g_selected_task = 0;

KeyEvent_t Get_Key_Event(void)
{
    static uint32_t key_down_tick = 0;
    static uint8_t  key_pressed = 0;
    uint32_t hold_time = 0;

    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0)
    {
        if (key_pressed == 0)
        {
            delay_ms(20);
            if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0)
            {
                key_pressed = 1;
                key_down_tick = HAL_GetTick();
            }
        }
    }
    else
    {
        if (key_pressed == 1)
        {
            key_pressed = 0;
            hold_time = HAL_GetTick() - key_down_tick;

            if (hold_time >= 1000)
            {
                return KEY_EVENT_LONG;
            }
            else if (hold_time >= 50)
            {
                return KEY_EVENT_SHORT;
            }
        }
    }
    return KEY_EVENT_NONE;
}

extern void Task1_Run(void);
extern void Task2_Run(void);
extern void Task3_Run(void);
extern void Task4_Run(void);
void System_Scheduler(void)
{
    SYN6658_Speak(TTS_PLEASE_SELECT);
    g_sys_state = SYS_STATE_TASK_SELECT;
    g_selected_task = 0;

    while (1)
    {
        if (g_sys_state == SYS_STATE_TASK_SELECT)
        {
            KeyEvent_t key = Get_Key_Event();

            if (key == KEY_EVENT_SHORT)
            {
                g_selected_task++;
                if(g_selected_task > 4) g_selected_task = 1;
                SYN6658_Speak(TASK_NAMES[g_selected_task]);
            }
            else if (key == KEY_EVENT_LONG)
            {
                if (g_selected_task == 0)
                {
                    SYN6658_Speak(TTS_NO_TASK_SELECTED);
                }
                else
                {
                    SYN6658_Speak(ENTER_TASK_NAMES[g_selected_task]);      
                    delay_ms(1500); // 等待语音播报结束
                    g_sys_state = SYS_STATE_TASK_RUNNING;
                }
            }
        }
        else if (g_sys_state == SYS_STATE_TASK_RUNNING)
        {
            switch (g_selected_task)
            {
                case 1: Task1_Run(); break;
                case 2: Task2_Run(); break;
                case 3: Task3_Run(); break;
                case 4: Task4_Run(); break;
            }
        }
    }
}

int main(void)
{
/* 1????????????????? */
SysTick_Config(SystemCoreClock / 1000); // ??????SysTick????HAL_GetTick????
delay_ms(1000);
/* ?????????NVIC????????????? */
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
/* ????????????? */
MX_TIM1_PWM_Init();           // 定时器1 PWM (PA8~PA11)
Motor_GPIO_Init();            // 电机引脚/接口

    /* 闭环必须：使能编码器与定时器 */
    MX_TIM2_Encoder_Init();
    MX_TIM3_Encoder_Init();
    MX_TIM6_Init();               // 核心计算定时器 (10ms)
    
    /* 速度环PID参数初始化 (Kp, Ki, Kd, OutMax, OutMin) */
    /* N20电机经验值：Kp=15.0, Ki=1.5, Kd=0.5; 增大PWM输出上限以增加转弯扭矩 */
    PID_Init(&g_pid_left, 15.0f, 1.5f, 0.5f, 950.0f, -950.0f);
    PID_Init(&g_pid_right, 15.0f, 1.5f, 0.5f, 950.0f, -950.0f);

    MX_USART1_Init();             // UART1 ??? (OpenMV), ????? OpenMV_Init() ???
MX_USART2_Init();             // ?????????? (PA2/PA3, 9600)
SYN6658_Init();               // ???????????
OpenMV_Init();                // OpenMV 初始化
IR_Init();                    // 5路红外模块
MX_USART3_Init();             // MPU6050模块串口
MPU6050_Init();               // 串口MPU数据接口
Alert_Init();                 // 声光报警初始化
StartButton_Init();           // 初始化PA15作为启动按键

/* ====== 上电完成提示 ====== */
SYN6658_Speak("\xC9\xCF\xB5\xE7\xCD\xEA\xB3\xC9"); // "上电完成"的GBK十六进制
Alert_Beep();                 // 蜂鸣器短促滴一声
/* ========================== */

/* ????? */
Motor_Stop();
Motor_Enable(1);
/* ==============================================
*                 ???????????????????????
* ============================================== */
    System_Scheduler();
}

/**
 * @brief  USART3 IRQ 
 */
void USART3_IRQHandler(void)
{
    uint8_t res;
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        res = USART_ReceiveData(USART3);
        MPU6050_UartParse(res);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
    /* ORE fix */
    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(USART3); /* Read DR to clear ORE */
    }
}
