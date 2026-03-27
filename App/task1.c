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

void Blink_LED_PC13(void)
{
    // 初�化PC13为输�
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // ��
    delay_ms(200);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);   // ��
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
    
    // 起点��示意，取消�音播报以防和“进入任务一”重叠�致冲突
    Blink_LED_PC13();
    
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
        // 1. 时时刻刻获取并更新传感器数据
        IR_Read(&g_ir_data);
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
        if (!passed_B && total_pulse >= 10000) 
        {
            SYN6658_ReportStation('B'); // 到达B�
            Blink_LED_PC13();
            passed_B = 1;
            // B点不重置，因为脉冲重�在C点进�
        }
        
        // 【到达C点�判�：�度达到170度绝对�，且��度小于15�/秒，且未曾播报过
        if (!passed_C && abs_yaw > 175.0f) 
        {
            SYN6658_ReportStation('C'); // 到达C�
            Blink_LED_PC13();
            passed_C = 1;
            
            // 核心要求：与此同时清空脉冲�数，重新�录
            total_pulse = 0; 
        }
        
        // 【到达D点�判�：在清空后续（即C点后），脉冲再一次大�9990且红�1/2/4/5任意��发生变化
        if (passed_C && !passed_D && total_pulse >= 10000) 
        {
            SYN6658_ReportStation('D'); // 到达D�
            Blink_LED_PC13();
            passed_D = 1;
        }
        
        // 【返回A点�终点判�：一直监测�度，�果达到350度绝对�且角�度小于15，�为终点
        if (abs_yaw > 360.0f) 
        {
            SYN6658_ReportStation('A'); // 回到A点停�
            Blink_LED_PC13();
            // 先将PID�标清零，取消��输出带来的抗拒干�
            g_pid_left.target = 0;
            g_pid_right.target = 0;
            Motor_Stop(); 
            
            // 任务完全结束，锁死保持停�
            while(1) {
                g_pid_left.target = 0;
                g_pid_right.target = 0;
                Motor_Stop();
                delay_ms(100);
            }
        }

        // 4. 寻线状�机逻辑完全解�并在主���同时执�
        SM_Process(&g_state_machine, event);
        Debug_LogIR(event);
        if (g_state_machine.current_state == STATE_LINE_FOLLOW)
        {
            LineFollow_RunByRawIR();
        }
        
        delay_ms(10); // 同�将主循�周期改回10ms
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
                Blink_LED_PC13();
                delay_ms(1500); 
            }
            OpenMV_ClearNewFlag(); 
        }
        delay_ms(10);
    }
}
