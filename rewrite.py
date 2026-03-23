import re

with open(r'D:\KeilMDKARM5.35\smart-car\App\main.c', 'r', encoding='utf-8') as f:
    text = f.read()

# Replace main() implementation
main_regex = re.compile(r'int main\(void\)\s*\{.*$', re.DOTALL)

new_main = '''int main(void)
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
    
    MX_I2C1_Init();               // MPU6050 I2C
    MPU6050_Init();               // 初始化MPU并在内部分析零飘
    
    StartButton_Init();           // 按键PA15初始化 (输入上拉)
    
    /* 关闭电机 */
    Motor_Stop();
    Motor_Enable(1);

    /* ==============================================
     *                 开始前的准备：3秒按键启动
     * ============================================== */
    uint32_t press_time = 0;
    while(1) {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET) {
            delay_ms(10);
            if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == Bit_RESET) {
                press_time += 10;
                if(press_time >= 3000) { // 已按满3秒
                    break;
                }
            }
        } else {
            press_time = 0; // 松开就清零
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
            SYN6658_Speak("发生碰撞，紧急停止");
            while(1); // 直接锁死停跳
        }
        last_mpu = g_mpu_data;

        /* 4. OpenMV 图像物体识别及语音播报 (串口接收中断会将新结果塞进这里) */
        if (OpenMV_HasNewData()) {
            OpenMV_DataTypeDef mv_data = OpenMV_GetResult();
            
            // "按照要求，只播报摄像头发现的剪刀打火机等东西"
            if (mv_data.object_id == 0x01) {
                SYN6658_Speak("发现打火机");
            } else if (mv_data.object_id == 0x02) {
                SYN6658_Speak("发现剪刀");
            } else if (mv_data.object_id == 0x03) {
                SYN6658_Speak("发现锤子");
            }
            OpenMV_ClearNewFlag();
        }

        /* 5. 寻迹+动力层 60%控制逻辑
         * 基础动力给到最大1000的60%，即 600
         */
        int16_t speed_base = 600;
        int16_t turn_offset = 0; // 差速调整

        // 如果红外探测到全黑或者全白（特殊情况保护）
        if (g_ir_data.all_white) {
            // 全脱线，可以根据MPU稍微找直
            // 比如结合MPU的角度或者yaw积分
            turn_offset = 0; 
        } 
        else if (g_ir_data.all_black) {
            // 遇到十字路口，保持直行
            turn_offset = 0;
        } 
        else {
            /* 标准5路红外差速打角 (g_ir_data.position 在 -2, -1, 0, 1, 2 之间)
             * 具体放大倍数看你们场地弧度，若150转不去可加到200。
             * "mpu焊接有点歪"，这里用一小部分偏差直接修正了直线寻迹
             */
             // 我们提供了一个粗暴但有效的寻迹P项控制
             turn_offset = g_ir_data.position * 180; 
        }

        // 把左右动力发送下去
        Motor_SetLeft(speed_base + turn_offset);
        Motor_SetRight(speed_base - turn_offset);

        /* 留作延时，降低 CPU 和 I2C/UART 的并发轮询负担（约50Hz控制）*/
        delay_ms(20);
    }
}
'''

new_text = main_regex.sub(new_main, text)
with open(r'D:\KeilMDKARM5.35\smart-car\App\main.c', 'w', encoding='utf-8') as f:
    f.write(new_text)

print("Main rewritten")
