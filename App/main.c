/**
 * @file    main.c
 * @brief   智能巡检小车 —— 主程序
 * @author  2026电子设计竞赛
 * @note    
 *   主控: STM32F407ZGT6 (标准外设库 SPL)
 *   视觉: OpenMV H7 Plus (UART1, 115200, PB6=TX, PB7=RX)
 *   语音: SYN6658 (UART2, 9600, PA2=TX, PA3=RX)
 *   电机: 4× N20 + 4× TB6612FNG (TIM1 PWM, PA8~PA11)
 *   编码器: 左=TIM2(PA0,PA1), 右=TIM3(PA6,PA7)
 *   巡线: 5路红外 (PB10~PB14)
 *   姿态: MPU6050 (I2C1, PB8=SCL, PB9=SDA)
 *   PID定时器: TIM6 (10ms中断)
 * 
 * 【移植与使用说明】
 *   本项目已完全脱离 CubeMX 和 HAL 库，采用纯代码硬写（STM32F4 标准外设库）配置：
 *   1. 准备一个纯净的 STM32F4 标准库工程模板（包含启动文件、CMSIS、STM32F4xx_StdPeriph_Driver）。
 *   2. 将本项目 App 文件夹下的所有 .c/.h 直接添加到 Keil 工程中。
 *   3. 在 Keil 魔术棒 C/C++ 选项卡添加头文件路径并在 Define 中填写 USE_STDPERIPH_DRIVER。
 *   4. 直接编译下载即可运行！不再需要使用 CubeMX 生成任何代码。
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
 *                     全局变量
 * ================================================================ */

/* PID控制器（左右轮独立） */
PID_TypeDef g_pid_left;
PID_TypeDef g_pid_right;

/* 转向PID（根据红外偏移量调整左右差速） */
PID_TypeDef g_pid_turn;

/* 红外传感器数据 */
IR_DataTypeDef g_ir_data;

/* MPU6050 数据 */
MPU6050_DataTypeDef g_mpu_data;

/* 状态机 */
StateMachine_TypeDef g_state_machine;

/* 编码器速度反馈 */
volatile int16_t g_encoder_left  = 0;
volatile int16_t g_encoder_right = 0;

/* 基础速度（巡线时的期望速度，编码器脉冲/10ms） */
int16_t g_base_speed = 15;

/* PID输出 */
volatile float g_motor_left_pwm  = 0;
volatile float g_motor_right_pwm = 0;

/* 10ms标志位 */
volatile uint8_t g_flag_10ms = 0;

/* ================================================================
 *                     工具延时函数（基于SysTick，粗略阻塞延时）
 * ================================================================ */
volatile uint32_t g_sys_tick = 0;

uint32_t HAL_GetTick(void)
{
    return g_sys_tick;
}

/* SysTick_Handler 已经在 stm32f4xx_it.c 中定义过了。
 * 如果你想在这里统一管理，请确保屏蔽或删除 stm32f4xx_it.c 中的 SysTick_Handler，
 * 或者把这段自增代码放入 stm32f4xx_it.c 的 SysTick_Handler 中。
 * 为了兼容模板，我们可以在这把它注释掉，用户去 stm32f4xx_it.c 里加一行 g_sys_tick++;
 */
// void SysTick_Handler(void)
// {
//     g_sys_tick++;
// }

void delay_ms(__IO uint32_t nTime)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = nTime;
    
    while((HAL_GetTick() - tickstart) < wait)
    {
    }
}

/* ================================================================
 *                     外设初始化函数 (标准库版)
 * ================================================================ */

/**
 * @brief  TIM1 PWM初始化 —— 电机驱动 (168MHz / 168 / 1000 = 1kHz)
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
 * @brief  TIM2 编码器模式 —— 左轮
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
 * @brief  TIM3 编码器模式 —— 右轮
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
 * @brief  TIM6 基本定时器 —— 10ms PID控制中断
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
 * @brief  UART1 初始化 —— OpenMV_TX/RX (PB6/PB7)
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
 * @brief  UART2 初始化 —— SYN6658 语音 (PA2/PA3)
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
 * @brief  I2C1 初始化 —— MPU6050 (PB8=SCL, PB9=SDA)
 */
static void MX_I2C1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef   I2C_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;

    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

/* ================================================================
 *                     中断服务函数与回调 (标准库底层)
 * ================================================================ */

/**
 * @brief  USART1 中断服务函数 —— OpenMV 数据接收
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t RxData = USART_ReceiveData(USART1);
        OpenMV_ParseByte(RxData); /* 标准库直接传入单字节或在内部处理接收缓冲 */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/**
 * @brief  TIM6 中断服务函数 —— 10ms PID定时器核心
 */
void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        /* ===== 1. 读取编码器（获取当前速度） ===== */
        g_encoder_left  = Encoder_Read_TIM2(); /* 按照标准库方式重写，比如取 TIM2->CNT */
        g_encoder_right = Encoder_Read_TIM3();

        /* ===== 2. 读取红外传感器 ===== */
        IR_Read(&g_ir_data);

        /* ===== 3. 读取MPU6050并更新偏航角 ===== */
        MPU6050_ReadAll(&g_mpu_data);
        MPU6050_UpdateYaw(&g_mpu_data, 0.01f);  /* dt = 10ms = 0.01s */

        /* ===== 4. 对状态机处于巡线模式进行控制 ===== */
        if (g_state_machine.current_state == STATE_LINE_FOLLOW)
        {
            float turn_output = 0; // 建议配合红外 g_ir_data.position 计算 turn_output
            float target_left  = (float)g_base_speed + turn_output;
            float target_right = (float)g_base_speed - turn_output;

            g_motor_left_pwm = PID_Incremental(&g_pid_left, target_left, (float)g_encoder_left);
            g_motor_right_pwm = PID_Incremental(&g_pid_right, target_right, (float)g_encoder_right);

            Motor_SetLeft((int16_t)g_motor_left_pwm);
            Motor_SetRight((int16_t)g_motor_right_pwm);
        }

        /* 设置全局标志位供 main 循环调度 */
        g_flag_10ms = 1;
    }
}

/* ================================================================
 *                     事件检测函数
 * ================================================================ */

/**
 * @brief  检测当前事件（供状态机使用）
 */
static CarEvent_TypeDef DetectEvent(void)
{
    if (g_ir_data.all_black)
    {
        return EVENT_CROSS_DETECTED;
    }
    if (OpenMV_HasNewData())
    {
        return EVENT_VISION_DONE;
    }
    return EVENT_NONE;
}

/* ================================================================
 *                     启动按键
 * ================================================================ */

/**
 * @brief  启动按键初始化（PA15，按下低电平）
 */
static void StartButton_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
 * @brief  按键检测 (带基本消抖)
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
 *                         主函数
 * ================================================================ */
int main(void)
{
    /* 标准库：配置NVIC中断优先级分组（2位置抢占优先，2位置响应优先） */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* 配置SysTick为1ms中断，用于延时和状态机心跳 */
    SysTick_Config(SystemCoreClock / 1000);

    /* ---------- 外设初始化 ---------- */
    Motor_GPIO_Init();
    MX_TIM1_PWM_Init();
    MX_TIM2_Encoder_Init();
    MX_TIM3_Encoder_Init();
    MX_USART1_Init();
    MX_USART2_Init();
    MX_I2C1_Init();
    IR_Init();
    Alert_Init();
    StartButton_Init();

    /* ---------- 模块初始化 ---------- */
    // 注意：对应的子模块内部应移除HAL串口或I2C句柄传递，改为直接传串口号USART1等
    OpenMV_Init();       
    SYN6658_Init();      
    MPU6050_Init();      

    /* ---------- PID 参数初始化 ---------- */
    PID_Init(&g_pid_left,  3.0f, 0.0f, 0.0f,  MOTOR_PWM_MAX, -MOTOR_PWM_MAX);
    PID_Init(&g_pid_right, 3.0f, 0.0f, 0.0f,  MOTOR_PWM_MAX, -MOTOR_PWM_MAX);
    PID_Init(&g_pid_turn,  80.0f, 0.0f, 20.0f, (float)g_base_speed, -(float)g_base_speed);

    /* ---------- 状态机初始化 ---------- */
    SM_Init(&g_state_machine, 5);

    /* ---------- 启动语音提示 ---------- */
    Alert_Beep();
    SYN6658_Speak("系统就绪，请按启动键");

    /* ---------- 启动系统控制调度（PID定时器等） ---------- */
    MX_TIM6_Init();

    /* ========== 主循环 ========== */
    while (1)
    {
        if (g_state_machine.current_state == STATE_IDLE)
        {
            if (StartButton_IsPressed())
            {
                SYN6658_Speak("小车启动");
                delay_ms(1000);
                PID_Reset(&g_pid_left);
                PID_Reset(&g_pid_right);
                PID_Reset(&g_pid_turn);
                MPU6050_ResetYaw(&g_mpu_data);
                SM_Process(&g_state_machine, EVENT_START);
            }
        }

        if (g_flag_10ms)
        {
            g_flag_10ms = 0;
            CarEvent_TypeDef event = DetectEvent();
            SM_Process(&g_state_machine, event);
        }
    }
}
