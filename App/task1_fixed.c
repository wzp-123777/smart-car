#include "task.h"
#include "state_machine.h"

#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "openmv.h"
#include "infrared.h"
#include "syn6658.h"
#include "pid.h"

extern StateMachine_TypeDef g_state_machine;
extern PID_TypeDef g_pid_left;
extern PID_TypeDef g_pid_right;
extern MPU6050_DataTypeDef g_mpu_data;
extern IR_DataTypeDef g_ir_data;
extern volatile int16_t g_encoder_left;
extern volatile int16_t g_encoder_right;
extern uint32_t HAL_GetTick(void);
extern void delay_ms(uint32_t);
extern void LineFollow_CalibrateGyroBias(void);
extern CarEvent_TypeDef DetectEvent(void);
extern void LineFollow_RunByRawIR(void);
extern void Debug_LogIR(CarEvent_TypeDef event);

void Task2_Run(void) { while(1) delay_ms(10); }
void Task4_Run(void) { while(1) delay_ms(10); }


/* ====== 任务1 状�机：A->B->C->D->A ====== */

int pc13_led_timer = 0;

void TurnOn_LED_PC13(void)
{
    static uint8_t init_done = 0;
    if (!init_done) {
        // 初始化PC13
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_SetBits(GPIOC, GPIO_Pin_13); // 默认高电平灭

        
        init_done = 1;
    }
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // PC13 低电平亮
    
    pc13_led_timer = 100; // 配合主循环非阻塞延迟：100*10ms = 1秒，充分点亮
}

void Task1_Run(void) 
{
    // 初�化原版��
    SM_Init(&g_state_machine, 3); 
    MPU6050_ResetYaw(&g_mpu_data);
    LineFollow_CalibrateGyroBias();
    IR_ResetTracking();
    SM_Process(&g_state_machine, EVENT_START); 
    
    // 初�化外层里程逻辑：�线程状态标�
    int32_t total_pulse = 0; 
    uint8_t passed_B = 0;
    uint8_t passed_C = 0;
    uint8_t passed_D = 0;
    float yaw_at_C = 0.0f; // 记录到达C点时的角度
    int delay_timer_C = 0; // C点延迟计时器
    uint8_t is_yaw_cleared_at_C = 0; // 标记C点后是否已经延迟1s清空了角度


    // 删除了此处的开机闪烁
    TurnOn_LED_PC13(); // 添加回开机提示，方便您测试LED是否正常亮起

    
    // 强制等待�下，防�电机启动过于突�带来干扰，同时�状态机预�
    delay_ms(500); 
    
    // 清除这期间可能积攒的OpenMV事件，避免��触发覆盖
    OpenMV_ClearNewFlag();

    // 更新初��度数据
    MPU6050_DataTypeDef last_mpu;
    MPU6050_ReadAll(&last_mpu);

    // 记录上一�的IR状�用于边缘判�
    uint8_t last_ir[5] = {0};
    IR_Read(&g_ir_data);
    for(int i=0; i<5; i++) last_ir[i] = g_ir_data.sensor[i];

    // 主循�
    while(1)
    {
        // 1. 时时刻刻获取并更新传感器数据 (IR_Read 已移入TIM6定时器中断以保证10ms实时性)
        MPU6050_ReadAll(&g_mpu_data);
        CarEvent_TypeDef event = DetectEvent(); 
        
        // 判断1�2�4�5路IR传感�（数组下�0,1,3,4）是否发生状态变�
        
                             
        // 更新last_ir供下�次判�
        for(int i=0; i<5; i++) last_ir[i] = g_ir_data.sensor[i];
        
        // 2. �立�持�地累加�算脉冲和绝对�度 (脱�阻塞判�)
        total_pulse += g_encoder_left; 
        
        float abs_yaw = g_mpu_data.yaw;
        if (abs_yaw < 0.0f) abs_yaw = -abs_yaw;
        
        // 取��度的绝对�，用于防抖判断
        float abs_gyro_z = g_mpu_data.gyro_z_dps;
        if (abs_gyro_z < 0.0f) abs_gyro_z = -abs_gyro_z;
        
        // 3. �立里程及角度条件判定（�线程并行�辑核心�
        
        // 【到达B点�判�：脉冲大�9990，且红�1/2/4/5任意��发生变化，且�曾播报过
        if (!passed_B && total_pulse >= 6856) 
        {
            SYN6658_ReportStation('B'); // 到达B�
            TurnOn_LED_PC13();
            passed_B = 1;
            // B点不重置，因为脉冲重�在C点进�
        }
        
        // 【到达C点�判�：�度达到170度绝对�，且��度小于15�/秒，且未曾播报过
        if (!passed_C && abs_yaw > 178.0f) 
        {
            SYN6658_ReportStation('C'); // 到达C�
            TurnOn_LED_PC13();
            passed_C = 1;
            // 核心要求：同时清空脉冲数，启动1s的延迟计时器（1s后清空角度基准）
            total_pulse = 0;
            delay_timer_C = 100; // 100 * 10ms = 1000ms = 1s
        }

        // C点后的非阻塞延时清空角度逻辑
        if (passed_C && !is_yaw_cleared_at_C)
        {
            if (delay_timer_C > 0)
            {
                delay_timer_C--;
            }
            else
            {
                yaw_at_C = g_mpu_data.yaw;
                is_yaw_cleared_at_C = 1;
            }
        }
        
        // 【到达D点�判�：在清空后续（即C点后），脉冲再一次大�9990且红�1/2/4/5任意��发生变化
        if (passed_C && !passed_D && total_pulse >= 6856) 
        {
            SYN6658_ReportStation('D'); // 到达D�
            TurnOn_LED_PC13();
            passed_D = 1;
        }
        
        // 【返回A点终点判：到达C点并过1s记录角度基准后，当前角度减去C点记录的角度，绝对值超过178度，则为到达A点
        float yaw_diff_from_C = g_mpu_data.yaw - yaw_at_C;
        if (yaw_diff_from_C < 0.0f) yaw_diff_from_C = -yaw_diff_from_C;
        if (is_yaw_cleared_at_C && yaw_diff_from_C >= 178.0f)
        {
            SYN6658_ReportStation('A'); // 回到A点停
            TurnOn_LED_PC13();
                    if (is_yaw_cleared_at_C && yaw_diff_from_C >= 178.0f)
        {
            SYN6658_ReportStation('A'); // 回到A点停
            TurnOn_LED_PC13();
            
            /* 【关键修改】：改变状态机的状态，告诉定时器停止循迹覆盖 */
            g_state_machine.current_state = STATE_IDLE; 
            
            // 先将PID 标清零，取消  输出带来的抗拒干 
            g_pid_left.target = 0;
            g_pid_right.target = 0;
            Motor_Stop();
            // 任务完全结束，锁死保持停 
            while(1) {
                g_pid_left.target = 0;
                g_pid_right.target = 0;
                Motor_Stop();
                delay_ms(100);
            }
        }
            // 先将PID 标清零，取消  输出带来的抗拒干 
            g_pid_left.target = 0;
            g_pid_right.target = 0;
            Motor_Stop();
            // 任务完全结束，锁死保持停 
            while(1) {
                g_pid_left.target = 0;
                g_pid_right.target = 0;
                Motor_Stop();
                delay_ms(100);
            }
        }

        // 4. 寻线状机逻辑完全解并在主同时执
        SM_Process(&g_state_machine, event);
        Debug_LogIR(event);
        if (g_state_machine.current_state == STATE_LINE_FOLLOW)
        {
            // 已将 LineFollow_RunByRawIR(); 移至 TIM6 定时器中断内，以保证控制不被阻塞
        }
        if (pc13_led_timer > 0) {
            pc13_led_timer--;
            if (pc13_led_timer == 0) {
                GPIO_SetBits(GPIOC, GPIO_Pin_13);  // PC13 高电平灭
            }
        }
        delay_ms(10); // 同将主循周期改回10ms
    }
}








void Task3_Run(void) 
{
    g_pid_left.target = 0;
    g_pid_right.target = 0;
    Motor_Stop();
    
    OpenMV_ClearNewFlag();
        while(1)
    {
        g_pid_left.target = 0;
        g_pid_right.target = 0;
        Motor_Stop();

        if (OpenMV_HasNewData())
        {
            OpenMV_DataTypeDef mv = OpenMV_GetResult();
            if (mv.is_valid && mv.object_id != 0) 
            {
                SYN6658_ReportObject(mv.object_id); 

                // 移除了 1500ms 的 delay_ms，改为单纯清空标志位
            }
            OpenMV_ClearNewFlag(); 
        }
        
        // 驱动 PC13 的非阻塞定时器
        if (pc13_led_timer > 0) {
            pc13_led_timer--;
            if (pc13_led_timer == 0) {
                GPIO_SetBits(GPIOC, GPIO_Pin_13);  // PC13 高电平灭
            }
        }
        
        delay_ms(10);
    }
}
