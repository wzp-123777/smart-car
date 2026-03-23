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

/* 使用简单的死循环延时，不依赖 SysTick 中断，防止中断死锁导致程序挂掉 */
void delay_ms(__IO uint32_t nTime)
{
    // 大概的死循环延时 (针对 168MHz 主频)
    __IO uint32_t i = 0;
    while(nTime--)
    {
        i = 25000;
        while(i--) ;
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
 * @brief  USART3 初始化 —— MPU6050 串口版 (PB10=TX, PB11=RX)
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

    USART_InitStructure.USART_BaudRate = 115200; // JY61常用波特率是115200或者9600
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
 * @brief  I2C1 初始化 —— MPU6050 (PB8=SCL, PB9=SDA)
 */


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
            // 【极其简单的纯开环测试：不依赖PID，固定输出转速】
            // 给一个固定 PWM 400（占空比40%），只要电机线没接错必转
            Motor_SetLeft(400);
            Motor_SetRight(400);
            
            // 【刹车测试】：如果用手指挡住**最左边(IR1)**的红外传感器
            // 此时反射光增强，通常红外模块输出低电平(0)
            if (g_ir_data.sensor[0] == 0) 
            {
                Motor_Stop(); // 触发断电滑行刹车
                g_state_machine.current_state = STATE_FINISHED; // 结束运行
            }
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

    /* PA15 默认是 JTAG 的 JTDI 引脚，如果不做特殊配置，无法作为普通按键使用！！ */
    // 开启按键和备用功能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // 不用像F1那样禁用JTAG，F4中只要把模式改为输入模式即可，但有些芯片可能有坑。
    // 但是这里最致命的是按键根本读不到状态，我们干脆把测试改为上电就跑！

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
    /* 1秒用于传感器上电稳定 */
    delay_ms(1000);

    /* 标准库：配置NVIC中断优先级分组 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 初始化所有外设 */
    MX_TIM1_PWM_Init();           // 定时器1 PWM (PA8~PA11)
    Motor_GPIO_Init();            // 电机方向/使能引脚

    // MX_TIM2_Encoder_Init();    // (目前采用开环PWM，可忽略编码器)
    // MX_TIM3_Encoder_Init();

    MX_USART2_Init();             // 语音通信复用 (PA2/PA3, 9600)
    SYN6658_Init();               // 语音模块初始化

    OpenMV_Init();                // OpenMV 通信 (UART1 115200 PB6/PB7)
    IR_Init();                    // 5路红外
    
    MX_USART3_Init();                // 初始化MPU6050串口 (PB10/PB11)
    
    StartButton_Init();           // 按键PA15初始化 (输入上拉)
    
    /* 关闭电机 */
    Motor_Stop();
    Motor_Enable(1);

    /* ==============================================
     *                 开始前的准备：按一下极速启动
     * ============================================== */
    while(1) {
        if(StartButton_IsPressed()) {
            break;  // 检测到按键按下并松开（已消抖），立刻跳出循环起跑
        }
    }
    
    /* 松手或者按满3秒后，播报一次上电完成 */
    SYN6658_Speak("上电完成");
    delay_ms(2000); // 给语音模块播报预留一点时间

    /* 记录开跑的系统绝对时间 */
    uint32_t start_run_time = HAL_GetTick();

    /* 记录上一周期MPU数据，用于算碰撞突变 */
    MPU6050_DataTypeDef last_mpu;
    MPU6050_ReadAll(&last_mpu);
    delay_ms(20);

    // MPU焊接有点歪，根据需求加入手动角度或坐标变换偏移
    // 假设我们在使用yaw时加个偏置或者修正
    float mpu_pitch_offset = 0.0f; 
    
    /* ========== 主控制循环 ========== */
    while (1)
    {
        uint32_t current_time = HAL_GetTick();

        /* 1. 运动十五秒后自动停止 */
        if (current_time - start_run_time >= 15000) {
            Motor_Stop();
            Motor_Enable(0);
            while(1); // 永远停在此处
        }

        /* 2. 读取当前红外和姿态、OpenMV数据 */
        IR_Read(&g_ir_data);
        MPU6050_ReadAll(&g_mpu_data);

        /* 3. 碰撞检测逻辑（基于 MPU 加速度突变）
         * 正常跑动加速度是1g(约16384Lsb)集中在Z轴，或者少部分在XY
         * 如果遇到急停碰撞，XY的瞬时差值会剧增，设置 8000 (约0.5G) 的安全碰撞阈值 */
        int16_t diff_x = g_mpu_data.accel_x - last_mpu.accel_x;
        int16_t diff_y = g_mpu_data.accel_y - last_mpu.accel_y;
        if (diff_x > 8000 || diff_x < -8000 || diff_y > 8000 || diff_y < -8000) {
            Motor_Stop();
            Motor_Enable(0);
            SYN6658_Speak("发生碰撞，紧急停止 ");
            while(1); // 直接锁死停跳
        }
        last_mpu = g_mpu_data;

        /* 4. OpenMV 图像物体识别及语音播报 (串口接收中断会将新结果塞进这里) */
        if (OpenMV_HasNewData()) {
            OpenMV_DataTypeDef mv_data = OpenMV_GetResult();
            
            // "按照要求，只播报摄像头发现的剪刀打火机等东西"
            if (mv_data.object_id == 0x01) {
                SYN6658_Speak("发现打火机 ");
            } else if (mv_data.object_id == 0x02) {
                SYN6658_Speak("发现剪刀 ");
            } else if (mv_data.object_id == 0x03) {
                SYN6658_Speak("发现锤子 ");
            }
            OpenMV_ClearNewFlag();
        }

        /* 5. 寻迹+动力层 60%控制逻辑
         * 修改：加入了PD控制避免画龙，并调低了过高的基础速度
         */
        int16_t speed_base = 300; // 30% 占空比 // 降低基础速度，如果太慢可以调回400或500
        int16_t turn_offset = 0; // 差速调整
        static int16_t last_position = 0; // 记录上次偏差用于PD控制
        
        // 【关键】如果你发现哪怕没有偏差(直走)小车也严重往右偏，可以给机械补偿
        // 例如：int16_t mech_offset = 20; (左轮快一点，右轮慢一点)
        int16_t mech_offset = 0; 

        // 如果红外探测到全黑或者全白（特殊情况保护）
        if (g_ir_data.all_white) {
            // 全脱线，可以根据MPU稍微找直
            turn_offset = 0;
            // 或者如果脱线需要停止： Motor_Stop(); while(1);
        }
        else if (g_ir_data.all_black) {
            // 遇到十字路口，保持直行
            turn_offset = 0;
        }
        else {
            /* 标准5路红外差速打角 PD控制避免"画龙" */
             // P比例：偏差越大，转向越猛（原代码180，这里稍降）
             int16_t Kp = 250; // 增大P，转弯更猛 
             // D微分(阻尼)：偏差变化越快，给个反向拉力，这是直线走得稳的灵魂！
             int16_t Kd = 450; // 增大D，保证大转角后能稳住 
             
             int16_t error = g_ir_data.position;
             turn_offset = (Kp * error) + Kd * (error - last_position);
             last_position = error;
        }

        // 把左右动力发送下去 (加入机械修正)
        int16_t final_left = speed_base + turn_offset + mech_offset;
        int16_t final_right = speed_base - turn_offset - mech_offset;
        
        // 限制在 -1000 到 1000 以内，防止溢出
        if(final_left > 1000) final_left = 1000;
        if(final_left < -1000) final_left = -1000;
        if(final_right > 1000) final_right = 1000;
        if(final_right < -1000) final_right = -1000;

        Motor_SetLeft(final_left);
        Motor_SetRight(final_right);
        /* 留作延时，降低 CPU 和 I2C/UART 的并发轮询负担（约50Hz控制）*/
        delay_ms(20);
    }
}

/**
 * @brief  USART3 中断服务函数 —— MPU6050 数据接收
 */
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t res = USART_ReceiveData(USART3);
        MPU6050_UartParse(res);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}
