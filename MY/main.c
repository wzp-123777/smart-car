#pragma diag_suppress 177
/**
* @file    main.c
* @brief   ??????????? ???? ??????
* @author  2026??????????
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
* ???????????????
*   ?????????????? CubeMX ?? HAL ????????????????STM32F4 ?????????????
*   1. ???????????? STM32F4 ????????????????????????CMSIS??STM32F4xx_StdPeriph_Driver????
*   2. ??????? App ????????????? .c/.h ???????? Keil ????????
*   3. ?? Keil ????? C/C++ ??????????????????? Define ?????? USE_STDPERIPH_DRIVER??
*   4. ????????????????????????????? CubeMX ???????????
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
*                     ??????
* ================================================================ */
/* PID??????????????????? */
PID_TypeDef g_pid_left;
PID_TypeDef g_pid_right;
/* ???PID?????????????????????????? */
PID_TypeDef g_pid_turn;
/* ???????????? */
IR_DataTypeDef g_ir_data;
/* MPU6050 ???? */
MPU6050_DataTypeDef g_mpu_data;
/* ???? */
StateMachine_TypeDef g_state_machine;
/* ???????????? */
volatile int16_t g_encoder_left  = 0;
volatile int16_t g_encoder_right = 0;
/* ????????????????????????????????/10ms?? */
int16_t g_base_speed = 15;
/* PID??? */
volatile float g_motor_left_pwm  = 0;
volatile float g_motor_right_pwm = 0;
/* 10ms????? */
volatile uint8_t g_flag_10ms = 0;

static int16_t g_line_follow_last_dir = 0;
static int16_t g_line_debug_left_pwm = 0;
static int16_t g_line_debug_right_pwm = 0;
static uint8_t g_line_debug_left_score = 0;
static uint8_t g_line_debug_right_score = 0;
static const char *g_line_debug_mode = "INIT";

#define LINE_DEBUG_INTERVAL_MS     150U
#define LINE_FOLLOW_LOOP_DELAY_MS    5U
#define LINE_FOLLOW_PWM_LIMIT      500
#define LINE_FOLLOW_SPEED_RUN      200
#define LINE_FOLLOW_SPEED_TURN     (-180)   /* 转弯修正时减速轮反相 */
#define LINE_FOLLOW_SPEED_CURVE_OUTER 380
#define LINE_FOLLOW_SPEED_EDGE_OUTER  500
#define LINE_FOLLOW_SPEED_SEARCH   420
#define LINE_FOLLOW_SPEED_WIDE     140
/* ================================================================
*                     ?????????????????SysTick???????????????
* ================================================================ */
volatile uint32_t g_sys_tick = 0;
uint32_t HAL_GetTick(void)
{
return g_sys_tick;
}
/* SysTick_Handler ????? stm32f4xx_it.c ??????????
* ??????????????????????????????????? stm32f4xx_it.c ???? SysTick_Handler??
* ??????????????????? stm32f4xx_it.c ?? SysTick_Handler ????
* ?????????????????????????????????? stm32f4xx_it.c ?????? g_sys_tick++;
*/
// void SysTick_Handler(void)
// {
//     g_sys_tick++;
// }
/* ??????????????????????? SysTick ???????????????????3????? */
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
         "IR bits=%u%u%u%u%u raw=0x%02X pos=%d aw=%u ab=%u ls=%u rs=%u lp=%d rp=%d mode=%s st=%s ev=%d\r\n",
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
         g_line_debug_mode,
         SM_GetStateName(g_state_machine.current_state),
         (int)event);
Debug_USART_SendString(USART3, buf);
}

static void Debug_LogText(const char *text)
{
Debug_USART_SendString(USART3, text);
}

static int16_t LineFollow_ClampPWM(int16_t pwm)
{
if (pwm < (-LINE_FOLLOW_PWM_LIMIT))
{
return -LINE_FOLLOW_PWM_LIMIT;
}
if (pwm > LINE_FOLLOW_PWM_LIMIT)
{
return LINE_FOLLOW_PWM_LIMIT;
}
return pwm;
}

static float track_kp = 2.0f;   /* 外环(位置环) 比例系数 - 依新精度微调 */
static float track_kd = 0.5f;   /* 外环(位置环) 微分系数 - 依新精度微调 */
static float track_last_error = 0.0f;
#define BASE_TARGET_SPEED 30.0f /* 基础目标速度（脉冲/10ms），可根据实际情况调节 */

static void LineFollow_RunByRawIR(void)
{
    float error = 0.0f;
    float p_out = 0.0f;
    float d_out = 0.0f;
    float total_adjust = 0.0f;
    float speed_left = BASE_TARGET_SPEED;
    float speed_right = BASE_TARGET_SPEED;
    uint8_t hit_count = 0;

    /* 
     * 传感器分布: [0]最左, [1]偏左, [2]中间, [3]偏右, [4]最右 
     * 根据要求：五路循迹精度放到10
     */
    if (g_ir_data.sensor[0]) { error -= 10.0f; hit_count++; }
    if (g_ir_data.sensor[1]) { error -= 5.0f;  hit_count++; }
    if (g_ir_data.sensor[2]) { error += 0.0f;  hit_count++; }
    if (g_ir_data.sensor[3]) { error += 5.0f;  hit_count++; }
    if (g_ir_data.sensor[4]) { error += 10.0f; hit_count++; }
    
    if (hit_count > 0) {
        error = error / (float)hit_count; /* 计算平均误差中心 */
    } else {
        /* 完全丢失黑线时，使用最后一次记录的误差极性放大 */
        if (track_last_error < 0.0f) { error = -12.0f; }
        else if (track_last_error > 0.0f) { error = 12.0f; }
        else { error = 0.0f; }
    }

    /* 外环：位置 PID 计算 */
    p_out = track_kp * error;
    d_out = track_kd * (error - track_last_error);
    total_adjust = p_out + d_out;
    
    /* 
     * 控制手段: 串级PID分配目标速度（允许分配到负值反相）
     * 策略：不加大外侧轮速度，只减小（或反向）内侧轮，以产生大差速转向。
     */
    if (total_adjust < 0.0f)
    {
        /* 偏左需左转：减小左侧内轮速度 */
        speed_left = BASE_TARGET_SPEED + total_adjust; // total_adjust为负，相加即减速/反转
        speed_right = BASE_TARGET_SPEED;
        g_line_debug_mode = "PID_L";
    }
    else if (total_adjust > 0.0f)
    {
        /* 偏右需右转：减小右侧内轮速度 */
        speed_left = BASE_TARGET_SPEED;
        speed_right = BASE_TARGET_SPEED - total_adjust;
        g_line_debug_mode = "PID_R";
    }
    else
    {
        speed_left = BASE_TARGET_SPEED;
        speed_right = BASE_TARGET_SPEED;
        g_line_debug_mode = "PID_C";
    }
    
    /* 更新上一拍误差 */
    if (hit_count > 0) track_last_error = error;

    /* 将计算出的分配速度送到内环（速度环）目标 */
    g_pid_left.target = speed_left;
    g_pid_right.target = speed_right;
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
        pwm_left = PID_Incremental(&g_pid_left, g_pid_left.target, (float)g_encoder_left);
        pwm_right = PID_Incremental(&g_pid_right, g_pid_right.target, (float)g_encoder_right);
        
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
/* 当前阶段只做循迹，不启用巡检点/视觉识别状态切换。 */
if (OpenMV_HasNewData())
{
OpenMV_ClearNewFlag();
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
    /* N20电机经验值：Kp=15.0, Ki=1.5, Kd=0.5; PWM全量程给1000或500对应最大值 */
    PID_Init(&g_pid_left, 15.0f, 1.5f, 0.5f, 500.0f, -500.0f);
    PID_Init(&g_pid_right, 15.0f, 1.5f, 0.5f, 500.0f, -500.0f);

    MX_USART1_Init();             // UART1 ??? (OpenMV), ????? OpenMV_Init() ???
MX_USART2_Init();             // ?????????? (PA2/PA3, 9600)
SYN6658_Init();               // ???????????
OpenMV_Init();                // OpenMV 初始化
IR_Init();                    // 5路红外模块
MX_USART3_Init();             // MPU6050模块串口
Alert_Init();                 // 声光报警初始化
StartButton_Init();           // 初始化PA15作为启动按键
/* ????? */
Motor_Stop();
Motor_Enable(1);
/* ==============================================
*                 ???????????????????????
* ============================================== */
/* ===== 状态机初始化 ===== */
SM_Init(&g_state_machine, 3); // 假设总共3个巡检点
Debug_LogText("DBG USART3 ready\r\n");
/* wait for start button */
while (StartButton_IsPressed() == 0)
{
IR_Read(&g_ir_data);
Debug_LogIR(EVENT_NONE);
delay_ms(10);
}

IR_ResetTracking();
IR_Read(&g_ir_data);
if (g_ir_data.position < 0)
{
g_line_follow_last_dir = -1;
}
else if (g_ir_data.position > 0)
{
g_line_follow_last_dir = 1;
}
else
{
g_line_follow_last_dir = 0;
}
/* ??????????3???????????????? */
SYN6658_Speak("??????");
delay_ms(2000); // ?????????????????????
/* ????????????????? */
uint32_t start_run_time = HAL_GetTick();
/* ??????????MPU????????????????? */
MPU6050_DataTypeDef last_mpu;
MPU6050_ReadAll(&last_mpu);
delay_ms(20);
// MPU?????????????????????????????????????
// ?????????????yaw??????????????
float mpu_pitch_offset = 0.0f;
// 驱动状态机
SM_Process(&g_state_machine, EVENT_START);
/* ========== 主循环 ========== */
while (1)
{
/* 1. 读取传感器 */
IR_Read(&g_ir_data);
MPU6050_ReadAll(&g_mpu_data);
/* 2. 事件检测：全黑、视觉识别完成 */
CarEvent_TypeDef event;
int16_t diff_x;
int16_t diff_y;
event = DetectEvent();
/* 3. 开环防倾倒保护（暂不停车，仅保留计算避免 unused 警告） */
diff_x = g_mpu_data.accel_x - last_mpu.accel_x;
diff_y = g_mpu_data.accel_y - last_mpu.accel_y;
(void)diff_x;
(void)diff_y;
last_mpu = g_mpu_data;
/* 4. 推动核心状态机运转 */
SM_Process(&g_state_machine, event);
Debug_LogIR(event);
/* 5. 按照状态机的当前状态执行对应动作 */
        if (g_state_machine.current_state == STATE_LINE_FOLLOW)
        {
            /* 规则式循迹：弯道优先降速，保证最大占空比不超过 50%。 */
            LineFollow_RunByRawIR();
        }
        delay_ms(LINE_FOLLOW_LOOP_DELAY_MS);
    }
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
