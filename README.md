# 智能巡检小车 —— 2026电子设计竞赛

## 📁 项目文件结构

```
smart-car/
└── App/
    ├── main.c              # 主程序（外设初始化 + 当前主循环控制）
    ├── pid.h / pid.c       # PID控制器（增量式 + 位置式）
    ├── motor.h / motor.c   # 电机驱动（TB6612FNG PWM控制）
    ├── encoder.h / encoder.c   # 编码器测速（定时器编码器模式）
    ├── infrared.h / infrared.c # 5路红外巡线传感器
    ├── openmv.h / openmv.c     # OpenMV串口通信协议
    ├── syn6658.h / syn6658.c   # SYN6658语音合成驱动
    ├── mpu6050.h / mpu6050.c   # 串口姿态模块驱动（当前按 USART3 方案）
    └── state_machine.h / state_machine.c  # 状态机框架
```

> 当前 `App/main.c` 走的是简化后的主循环方案：OpenMV 主动上报识别结果，巡线使用直接 PD 调整，编码器/PID/状态机代码仍保留在仓库中但默认未启用。
> 当前 `main.c` 默认自动启动；`PA15` 的启动按键代码仍保留在文件里，后续如需恢复按键启动可直接接回等待逻辑。

## 🔌 硬件引脚分配表（已解决所有冲突 ✅）

| 功能 | 外设 | 引脚 | 说明 |
|------|------|------|------|
| 电机1 PWM (左前) | TIM1_CH1 | PA8 | PWM频率1kHz |
| 电机2 PWM (右前) | TIM1_CH2 | PA9 | PWM频率1kHz |
| 电机3 PWM (左后) | TIM1_CH3 | PA10 | PWM频率1kHz |
| 电机4 PWM (右后) | TIM1_CH4 | PA11 | PWM频率1kHz |
| 电机1 方向 | GPIO | PC0, PC1 | IN1, IN2 |
| 电机2 方向 | GPIO | PC2, PC3 | IN1, IN2 |
| 电机3 方向 | GPIO | PC4, PC5 | IN1, IN2 |
| 电机4 方向 | GPIO | PC6, PC7 | IN1, IN2 |
| 左轮编码器 | TIM2 CH1/CH2 | PA0, PA1 | 编码器模式 |
| 右轮编码器 | TIM3 CH1/CH2 | PA6, PA7 | 编码器模式 |
| OpenMV通信 | USART1 | **PB6**(TX), **PB7**(RX) | 115200bps |
| SYN6658语音 | USART2 | PA2(TX), PA3(RX) | 9600bps |
| 串口姿态模块 | USART3 | PB10(TX), PB11(RX) | 当前工程按串口 IMU/JY61 类模块处理 |
| 红外传感器1~5 | GPIO Input | PD1~PD5 | 当前代码实际使用的 5 路巡线输入 |
| 启动按键 | GPIO Input | PA15 | 上拉，按下低电平 |

> 当前代码实际占用关系：`TIM1 -> PA8~PA11`，`OpenMV USART1 -> PB6/PB7`，`串口姿态模块 USART3 -> PB10/PB11`，`红外 -> PD1~PD5`。

## ⏱️ 定时器分配

| 定时器 | 用途 | 频率/参数 |
|--------|------|-----------|
| TIM1 | 4路PWM输出 | 168MHz/168/1000 = 1kHz |
| TIM2 | 左轮编码器 | 编码器接口模式 |
| TIM3 | 右轮编码器 | 编码器接口模式 |
| TIM6 | PID定时中断 | 84MHz/840/1000 = 100Hz(10ms)，保留框架，当前主循环未启用 |

## 🔧 移植到Keil工程步骤

### 方式一：使用CubeMX（推荐）

1. **创建CubeMX工程**
   - 芯片选择 STM32F407ZGT6
   - 配置外设：TIM1(PWM), TIM2/3(Encoder), TIM6(基本定时器), USART1, USART2, USART3
   - 使能中断：TIM6, USART1
   - 生成Keil工程

2. **添加App文件**
   - 将 `App/` 文件夹中的所有 `.c` 和 `.h` 文件添加到Keil工程
   - 在 Keil 的 Include Path 中添加 `App/` 路径

3. **修改CubeMX生成的main.c**
   - 在 `/* USER CODE BEGIN 2 */` 处调用各模块初始化
   - 在 `/* USER CODE BEGIN 3 */` 处放置主循环代码
   - 或者直接用本项目的 `main.c` 替换（保留CubeMX的HAL初始化代码）

### 方式二：手动配置

1. 新建Keil工程，选择STM32F407ZGT6
2. 添加HAL库驱动文件
3. 添加 `App/` 下所有源文件
4. 编译烧录

## 🎯 PID 调参指南

### 调参步骤（非常重要！）

#### 第一步：调速度PID（g_pid_left / g_pid_right）

```
初始参数: Kp=10.0, Ki=0.0, Kd=0.0
```

1. **只调Kp**：从5.0开始，逐步增大
   - Kp太小 → 电机响应慢，跟不上目标速度
   - Kp太大 → 电机抖动、震荡
   - 找到临界震荡点，取60%~70%作为Kp

2. **加入Ki**：从0.5开始
   - Ki消除稳态误差
   - Ki太大 → 超调、震荡
   - 一般 Ki = Kp × 0.1

3. **微调Kd**：从0.1开始（可选）
   - Kd抑制超调
   - Kd太大 → 高频抖动

#### 第二步：调转向PID（g_pid_turn）

```
初始参数: Kp=80.0, Ki=0.0, Kd=20.0
```

1. 只用P控制（Kp=80, Ki=0, Kd=0），观察巡线效果
2. 如果弯道跟踪不好，增加Kd
3. **转向PID一般不需要积分项（Ki=0）**

### 参考参数范围

| PID | Kp | Ki | Kd | 说明 |
|-----|----|----|-----|------|
| 速度PID | 5~20 | 0.5~3.0 | 0~2.0 | 左右轮独立 |
| 转向PID | 50~150 | 0 | 10~50 | 基于红外偏移量 |

### 调试技巧

1. **串口打印**：在10ms中断中通过串口打印编码器值和PID输出，用串口助手观察波形
2. **逐步增加速度**：先用低速(g_base_speed=15)调好PID，再逐步提速
3. **单轮测试**：先断开一侧电机，只测试一个轮子的PID效果
4. **直线测试**：PID调好后，先跑直线确认左右轮速度一致

## 📡 通信协议说明

### STM32 → OpenMV（请求帧）

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | 0xAA | 帧头 |
| 1 | CMD | 0x01=检测, 0x02=停止, 0x03=颜色, 0x04=形状, 0x05=二维码 |
| 2 | CMD | 校验和(=CMD) |
| 3 | 0x55 | 帧尾 |

### OpenMV → STM32（应答帧）

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | 0xBB | 帧头 |
| 1 | ID | 物体ID |
| 2 | Xh | X坐标高字节 |
| 3 | Xl | X坐标低字节 |
| 4 | Yh | Y坐标高字节 |
| 5 | Yl | Y坐标低字节 |
| 6 | CHK | 校验和 = (ID+Xh+Xl+Yh+Yl) & 0xFF |
| 7 | 0x55 | 帧尾 |

> 当前仓库里的 [`OpenMV/main.py`](/D:/KeilMDKARM5.35/smart-car/OpenMV/main.py) 采用“主动识别、主动上报”模式，STM32 侧主要负责接收结果；`App/openmv.c` 中保留了命令帧接口，便于后续切回按需触发。

## 🔄 状态机流程

```
[IDLE] ──按下启动键──→ [LINE_FOLLOW] ──检测到标记线──→ [STOP_AND_DETECT]
                            ↑                              │ (等待500ms)
                            │                              ↓
                            │                      [VISION_DETECT]
                            │                       │ (发送检测指令给OpenMV)
                            │                       ↓
                            │                   [VOICE_REPORT]
                            │                    │ (SYN6658播报结果)
                            │                    ↓
                            └──播报完成，继续巡线──┘
                                                 │
                                           所有点完成
                                                 ↓
                                           [FINISHED]
```

## 🏁 比赛现场快速修改清单

| 需要修改的内容 | 修改位置 | 说明 |
|---------------|----------|------|
| 巡检点数量 | main.c → `SM_Init(&g_state_machine, 5)` | 修改数字即可 |
| 基础速度 | main.c → `g_base_speed = 30` | 增大提速 |
| 速度PID参数 | main.c → `PID_Init(&g_pid_left, ...)` | 调Kp/Ki/Kd |
| 转向PID参数 | main.c → `PID_Init(&g_pid_turn, ...)` | 调Kp/Kd |
| 物体名称 | syn6658.c → `SYN6658_ReportObject()` | 修改播报文本 |
| 物体ID映射 | openmv.h → `OBJ_xxx` 宏定义 | 与OpenMV端对应 |
| 停车判断条件 | main.c → `DetectEvent()` | 修改标记检测逻辑 |
| 红外传感器极性 | infrared.c → `IR_Read()` | 取反读取值 |

## ⚡ 注意事项

1. **编码器方向**：如果电机正转但编码器计数为负，交换编码器A/B相接线
2. **红外极性**：不同品牌传感器高低电平可能相反，在 `IR_Read()` 中取反
3. **姿态模块**：当前工程按串口姿态模块处理，不是 I2C 版 MPU6050 驱动
4. **语音延时**：SYN6658 播报需要时间，主循环里仍保留了阻塞等待
5. **串口占用**：`USART1` 给 OpenMV，`USART2` 给 SYN6658，`USART3` 给姿态模块
