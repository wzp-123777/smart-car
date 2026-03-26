import os

file_path = r'd:\KeilMDKARM5.35\smart-car\App\task1.c'
content = '''#include "task.h"
#include "stm32f4xx.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "openmv.h"
#include "infrared.h"
#include "syn6658.h"
#include "alert.h"
#include <math.h>

extern StateMachine_TypeDef g_state_machine;
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

extern PID_TypeDef g_pid_left;
extern PID_TypeDef g_pid_right;

extern volatile int32_t g_distance_total; 

void Task2_Run(void) { while(1) delay_ms(10); }
void Task3_Run(void) { while(1) delay_ms(10); }
void Task4_Run(void) { while(1) delay_ms(10); }

/* ====== 任务1 状态机：A->B->C->D->A ====== */
typedef enum {
    M_STAGE_A_TO_B = 0,
    M_STAGE_B_TO_C,
    M_STAGE_C_TO_D,
    M_STAGE_D_TO_A,
    M_STAGE_FINISH
} MissionStage_t;

void Blink_LED_PC13(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC, GPIO_Pin_13); 
    delay_ms(200);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);   
}

float get_angle_diff(float a, float b) {
    float diff = a - b;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

void Task1_Run(void)
{
    MissionStage_t m_stage = M_STAGE_A_TO_B;
    const int32_t PULSES_PER_100CM = 2591; 
    float start_yaw = 0.0f;
    
    SM_Init(&g_state_machine, 3);
    MPU6050_ResetYaw(&g_mpu_data);
    LineFollow_CalibrateGyroBias();
    IR_ResetTracking();
    SM_Process(&g_state_machine, EVENT_START);

    g_distance_total = 0;

    SYN6658_Speak("\\xC6\\xF0\\xB5\\xE3\\x41\\xB5\\xE3"); // "起点A点"
    Blink_LED_PC13();

    while(1)
    {
        CarEvent_TypeDef event;
        IR_Read(&g_ir_data);
        MPU6050_ReadAll(&g_mpu_data);
        event = DetectEvent();

        if (m_stage == M_STAGE_A_TO_B && g_distance_total > PULSES_PER_100CM)
        {
            SYN6658_Speak("\\xB5\\xBD\\xB4\\xEF\\x42\\xB5\\xE3"); // "到达B点"
            Blink_LED_PC13();
            m_stage = M_STAGE_B_TO_C;
            start_yaw = g_mpu_data.yaw; 
            g_distance_total = 0;
        }
        else if (m_stage == M_STAGE_B_TO_C)
        {
            // B到C依赖循迹行驶，累计角度转过180度（加入少许容差大于175度即可）即为C点
            if (fabs(get_angle_diff(g_mpu_data.yaw, start_yaw)) > 175.0f) 
            {
                SYN6658_Speak("\\xB5\\xBD\\xB4\\xEF\\x43\\xB5\\xE3"); // "到达C点"
                Blink_LED_PC13();
                m_stage = M_STAGE_C_TO_D;
                g_distance_total = 0; 
            }
        }
        else if (m_stage == M_STAGE_C_TO_D && g_distance_total > PULSES_PER_100CM)
        {
            SYN6658_Speak("\\xB5\\xBD\\xB4\\xEF\\x44\\xB5\\xE3"); // "到达D点"
            Blink_LED_PC13();
            m_stage = M_STAGE_D_TO_A;
            start_yaw = g_mpu_data.yaw; 
            g_distance_total = 0;
        }
        else if (m_stage == M_STAGE_D_TO_A)
        {
            if (fabs(get_angle_diff(g_mpu_data.yaw, start_yaw)) > 175.0f) 
            {
                SYN6658_Speak("\\xBB\\xD8\\xB5\\xBD\\x41\\xB5\\xE3"); // "回到A点停车"
                Blink_LED_PC13();
                Motor_SetLeft(0);
                Motor_SetRight(0);
                g_pid_left.target = 0;
                g_pid_right.target = 0;
                m_stage = M_STAGE_FINISH;
            }
        }

        if (m_stage != M_STAGE_FINISH)
        {
            SM_Process(&g_state_machine, event);
            Debug_LogIR(event);
            if (g_state_machine.current_state == STATE_LINE_FOLLOW)
            {
                LineFollow_RunByRawIR();
            }
        }

        delay_ms(10);
    }
}
'''
with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)
