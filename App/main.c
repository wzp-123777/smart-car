/**
 * @file    main.c
 * @brief   智能巡检小车 —— 主程序
 * @author  2026电子设计竞赛
 * @note    
 *   主控: STM32F407ZGT6 (HAL库 + Keil)
 *   视觉: OpenMV H7 Plus (UART1, 115200, PB6=TX, PB7=RX)
 *   语音: SYN6658 (UART2, 9600, PA2=TX, PA3=RX)
 *   电机: 4× N20 + 4× TB6612FNG (TIM1 PWM, PA8~PA11)
 *   编码器: 左=TIM2(PA0,PA1), 右=TIM3(PA6,PA7)
 *   巡线: 5路红外 (PB10~PB14)
 *   姿态: MPU6050 (I2C1, PB8=SCL, PB9=SDA)
 *   PID定时器: TIM6 (10ms中断)
 * 
 * 【硬件定时器分配】
 *   TIM1:  PWM输出（4路电机驱动）  168MHz / 168 / 1000 = 1kHz PWM
 *   TIM2:  左侧编码器接口模式
 *   TIM3:  右侧编码器接口模式
 *   TIM6:  10ms基本定时器（PID计算+编码器采集+巡线读取）
 * 
 * 【移植步骤】
 *   1. 在CubeMX中配置好所有外设（UART1/2, TIM1/2/3/6, I2C1, GPIO）
 *   2. 将 App 文件夹中的 .c/.h 文件添加到 Keil 工程
 *   3. 在CubeMX生成的 main.c 中，调用本文件的初始化和主循环函数
 *   4. 或者直接替换 main.c（注意保留CubeMX的HAL初始化代码）
 */

#include "stm32f4xx_hal.h"
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
 *                     外设句柄声明
 * ================================================================ */
TIM_HandleTypeDef htim1;    /* PWM —— 电机驱动 */
TIM_HandleTypeDef htim2;    /* 编码器 —— 左轮 */
TIM_HandleTypeDef htim3;    /* 编码器 —— 右轮 */
TIM_HandleTypeDef htim6;    /* 基本定时器 —— 10ms PID中断 */
UART_HandleTypeDef huart1;  /* OpenMV 通信 */
UART_HandleTypeDef huart2;  /* SYN6658 语音 */
I2C_HandleTypeDef hi2c1;    /* MPU6050 陀螺仪 */

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
int16_t g_base_speed = 30;

/* PID输出 */
volatile float g_motor_left_pwm  = 0;
volatile float g_motor_right_pwm = 0;

/* 10ms标志位 */
volatile uint8_t g_flag_10ms = 0;

/* ================================================================
 *                     外设初始化函数
 * ================================================================ */

/**
 * @brief  系统时钟配置 (168MHz)
 * @note   如果使用CubeMX生成的SystemClock_Config，删除此函数
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
 * @brief  TIM1 PWM初始化 —— 电机驱动
 * @note   168MHz / PSC(168) / ARR(1000) = 1kHz PWM频率
 */
static void MX_TIM1_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 168 - 1;        /* 168MHz / 168 = 1MHz */
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1000 - 1;           /* 1MHz / 1000 = 1kHz */
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    /* 配置4个PWM通道 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

    /* 启动4路PWM输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* ===== PWM引脚GPIO配置 ===== */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  TIM2 编码器模式 —— 左轮
 * @note   PA0(TIM2_CH1), PA1(TIM2_CH2)
 */
static void MX_TIM2_Encoder_Init(void)
{
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 6;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 6;
    HAL_TIM_Encoder_Init(&htim2, &sEncoderConfig);

    /* 编码器引脚 GPIO */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Encoder_Init(&htim2);
}

/**
 * @brief  TIM3 编码器模式 —— 右轮
 * @note   PA6(TIM3_CH1), PA7(TIM3_CH2)
 */
static void MX_TIM3_Encoder_Init(void)
{
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 6;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 6;
    HAL_TIM_Encoder_Init(&htim3, &sEncoderConfig);

    /* 编码器引脚 GPIO */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Encoder_Init(&htim3);
}

/**
 * @brief  TIM6 基本定时器 —— 10ms PID中断
 * @note   84MHz(APB1) / 840 / 1000 = 100Hz = 10ms
 */
static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 840 - 1;        /* 84MHz / 840 = 100kHz */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000 - 1;           /* 100kHz / 1000 = 100Hz = 10ms */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim6);

    /* 使能TIM6中断 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* 启动定时器中断 */
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief  UART1 初始化 —— OpenMV (115200)
 * @note   使用 PB6(TX) / PB7(RX)，避免与 TIM1(PA8~PA11) 引脚冲突
 */
static void MX_USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6=TX, PB7=RX（USART1 复用映射到 PB6/PB7） */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* 使能UART1接收中断 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief  UART2 初始化 —— SYN6658 语音 (9600)
 */
static void MX_USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA2=TX, PA3=RX */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/**
 * @brief  I2C1 初始化 —— MPU6050 (PB8=SCL, PB9=SDA)
 */
static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;         /* 400kHz 快速模式 */
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* ================================================================
 *                     中断处理函数
 * ================================================================ */

/**
 * @brief  TIM6 中断服务函数 —— 10ms PID定时器
 * @note   如果使用CubeMX，此函数在 stm32f4xx_it.c 中
 *         这里提供完整实现，移植时选择一处即可
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/**
 * @brief  USART1 中断服务函数 —— OpenMV
 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief  定时器周期完成回调 —— 10ms PID控制核心
 * @note   这是整个小车控制的核心！每10ms执行一次：
 *         1. 读取编码器 → 获取当前速度
 *         2. 读取红外传感器 → 获取偏移量
 *         3. 读取MPU6050 → 获取姿态
 *         4. PID计算 → 输出PWM
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* ===== 1. 读取编码器（获取当前速度） ===== */
        g_encoder_left  = Encoder_Read(&htim2);
        g_encoder_right = Encoder_Read(&htim3);

        /* ===== 2. 读取红外传感器 ===== */
        IR_Read(&g_ir_data);

        /* ===== 3. 读取MPU6050并更新偏航角 ===== */
        MPU6050_ReadAll(&hi2c1, &g_mpu_data);
        MPU6050_UpdateYaw(&g_mpu_data, 0.01f);  /* dt = 10ms = 0.01s */

        /* ===== 4. 仅在巡线模式下进行PID控制 ===== */
        if (g_state_machine.current_state == STATE_LINE_FOLLOW)
        {
            /*
             * 转向策略：
             *   turn_output = PID_turn(0, ir_position)
             *   左轮速度 = base_speed + turn_output
             *   右轮速度 = base_speed - turn_output
             * 
             * 偏移量为正(偏右) → turn_output为负 → 左轮减速右轮加速 → 右转纠偏
             * 偏移量为负(偏左) → turn_output为正 → 左轮加速右轮减速 → 左转纠偏
             */
            float turn_output = PID_Incremental(&g_pid_turn, 0, (float)g_ir_data.position);

            float target_left  = (float)g_base_speed + turn_output;
            float target_right = (float)g_base_speed - turn_output;

            /* 左轮速度PID */
            g_motor_left_pwm = PID_Incremental(&g_pid_left, target_left,
                                                (float)g_encoder_left);

            /* 右轮速度PID */
            g_motor_right_pwm = PID_Incremental(&g_pid_right, target_right,
                                                 (float)g_encoder_right);

            /* 输出到电机 */
            Motor_SetLeft(&htim1, (int16_t)g_motor_left_pwm);
            Motor_SetRight(&htim1, (int16_t)g_motor_right_pwm);
        }

        /* 设置10ms标志（供main循环使用） */
        g_flag_10ms = 1;
    }
}

/**
 * @brief  串口接收完成回调
 * @note   HAL库会自动调用，处理所有UART的接收中断
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* OpenMV 数据接收 */
    OpenMV_UART_RxCallback(huart);
}

/* ================================================================
 *                     事件检测函数
 * ================================================================ */

/**
 * @brief  检测当前事件（供状态机使用）
 * @retval 当前发生的事件
 */
static CarEvent_TypeDef DetectEvent(void)
{
    /* 检测十字路口（5路传感器全黑） */
    if (g_ir_data.all_black)
    {
        return EVENT_CROSS_DETECTED;
    }

    /* 检测OpenMV是否返回了新数据 */
    if (OpenMV_HasNewData())
    {
        return EVENT_VISION_DONE;
    }

    /* 其他事件可以在这里添加 */

    return EVENT_NONE;
}

/* ================================================================
 *                     启动按键
 * ================================================================ */

/**
 * @brief  启动按键初始化（假设接在 PE0，按下为低电平）
 */
static void StartButton_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/**
 * @brief  检测启动按键（带简单消抖）
 */
static uint8_t StartButton_IsPressed(void)
{
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);  /* 消抖 */
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET);  /* 等待松手 */
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
    /* ---------- HAL 初始化 ---------- */
    HAL_Init();
    SystemClock_Config();

    /* ---------- 外设初始化 ---------- */
    Motor_GPIO_Init();          /* 电机方向引脚 */
    MX_TIM1_PWM_Init();         /* PWM定时器 */
    MX_TIM2_Encoder_Init();     /* 左轮编码器 */
    MX_TIM3_Encoder_Init();     /* 右轮编码器 */
    MX_USART1_Init();           /* OpenMV串口 */
    MX_USART2_Init();           /* SYN6658串口 */
    MX_I2C1_Init();             /* MPU6050 I2C */
    IR_Init();                  /* 红外传感器 */
    Alert_Init();               /* LED + 蜂鸣器 */
    StartButton_Init();         /* 启动按键 */

    /* ---------- 模块初始化 ---------- */
    OpenMV_Init(&huart1);       /* OpenMV通信 */
    SYN6658_Init(&huart2);      /* 语音模块 */
    MPU6050_Init(&hi2c1);       /* 陀螺仪 */

    /* ---------- PID 参数初始化 ---------- */
    /*
     * 【重要】以下PID参数需要根据实际电机特性调整！
     * 调参步骤见 pid.c 文件头部注释
     * 
     * 左/右轮速度PID：
     *   Kp=10.0  Ki=1.0  Kd=0.5
     *   输出范围: -1000 ~ +1000 (对应PWM占空比)
     */
    PID_Init(&g_pid_left,  10.0f, 1.0f, 0.5f,  MOTOR_PWM_MAX, -MOTOR_PWM_MAX);
    PID_Init(&g_pid_right, 10.0f, 1.0f, 0.5f,  MOTOR_PWM_MAX, -MOTOR_PWM_MAX);

    /*
     * 转向PID：
     *   Kp=80.0  Ki=0.0  Kd=20.0
     *   输出范围: -基础速度 ~ +基础速度
     *   注意: 转向PID一般不需要积分项
     */
    PID_Init(&g_pid_turn,  80.0f, 0.0f, 20.0f,
             (float)g_base_speed, -(float)g_base_speed);

    /* ---------- 状态机初始化 ---------- */
    SM_Init(&g_state_machine, 5);  /* 假设5个巡检点，根据赛题修改 */

    /* ---------- 启动语音提示 ---------- */
    Alert_Beep();               /* 滴一声表示上电成功 */
    SYN6658_Speak(&huart2, "系统就绪，请按启动键");

    /* ---------- 启动10ms定时器 ---------- */
    MX_TIM6_Init();

    /* ========== 主循环 ========== */
    while (1)
    {
        /* --- 检测启动按键 --- */
        if (g_state_machine.current_state == STATE_IDLE)
        {
            if (StartButton_IsPressed())
            {
                SYN6658_Speak(&huart2, "小车启动");
                HAL_Delay(1000);
                PID_Reset(&g_pid_left);
                PID_Reset(&g_pid_right);
                PID_Reset(&g_pid_turn);
                MPU6050_ResetYaw(&g_mpu_data);
                SM_Process(&g_state_machine, EVENT_START);
            }
        }

        /* --- 10ms周期任务（与定时器中断同步） --- */
        if (g_flag_10ms)
        {
            g_flag_10ms = 0;

            /* 检测事件 */
            CarEvent_TypeDef event = DetectEvent();

            /* 状态机处理 */
            SM_Process(&g_state_machine, event);
        }
    }
}
