#include "task.h"
#include "state_machine.h"

#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "openmv.h"
#include "infrared.h"
#include "syn6658.h"
#include "alert.h"

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

void Task2_Run(void) { while(1) delay_ms(10); }
void Task3_Run(void) { while(1) delay_ms(10); }
void Task4_Run(void) { while(1) delay_ms(10); }


/* ====== 任务1 状态机：A->B->C->D->A ====== */
typedef enum {
    M_STAGE_A_TO_B = 0,
    M_STAGE_B_TURN,
    M_STAGE_B_TO_C,
    M_STAGE_C_TURN,
    M_STAGE_C_TO_D,
    M_STAGE_D_TURN,
    M_STAGE_D_TO_A,
    M_STAGE_FINISH
} MissionStage_t;

void Blink_LED_PC13(void)
{
    // 初始化PC13为输出
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 亮灯
    delay_ms(200);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);   // 灭灯
}

void Task1_Run(void) 
{
    // 初始化原版所需
    SM_Init(&g_state_machine, 3); 
    MPU6050_ResetYaw(&g_mpu_data);
    LineFollow_CalibrateGyroBias();
    IR_ResetTracking();
    SM_Process(&g_state_machine, EVENT_START); 
    
    // 初始化外层里程逻辑
    MissionStage_t m_stage = M_STAGE_A_TO_B;
    int32_t total_distance = 0; // 用编码器累加表示距离
    
    // 起点A播报
    SYN6658_Speak("\xC6\xF0\xB5\xE3\x41\xB5\xE3"); // "起点A点"
    Blink_LED_PC13();
    
    MPU6050_DataTypeDef last_mpu;
    MPU6050_ReadAll(&last_mpu);

    // 主循环
    while(1)
    {
        IR_Read(&g_ir_data);
        MPU6050_ReadAll(&g_mpu_data);
        
        CarEvent_TypeDef event = DetectEvent(); 
        
        // 累加左轮作为参考里程 (需根据您的N20编码器线数修正这个阈值，这里假设大约100对应100cm，请实际测试更改)
        total_distance += g_encoder_left; 
        
        /* 外层任务1逻辑监控 */
        if (m_stage == M_STAGE_A_TO_B && total_distance > 2591) // 编码器达到2591对应到达B
        {
            SYN6658_Speak("\xB5\xBD\xB4\xEF\x42\xB5\xE3"); // 到达B点
            Blink_LED_PC13();
            m_stage = M_STAGE_B_TURN;
            total_distance = 0;
            MPU6050_ResetYaw(&g_mpu_data); // 重置Yaw偏航角用于转弯积分
        }
        else if (m_stage == M_STAGE_B_TURN) 
        {
            // B -> C 是弯道，检测偏航角是否达到180度
            if (g_mpu_data.yaw >= 180.0f || g_mpu_data.yaw <= -180.0f)
            {
                SYN6658_Speak("\xB5\xBD\xB4\xEF\x43\xB5\xE3"); // 到达C点
                Blink_LED_PC13();
                m_stage = M_STAGE_C_TO_D;
                total_distance = 0;
                MPU6050_ResetYaw(&g_mpu_data);
            }
        }
        else if (m_stage == M_STAGE_C_TO_D && total_distance > 2591) 
        {
            SYN6658_Speak("\xB5\xBD\xB4\xEF\x44\xB5\xE3"); // 到达D点
            Blink_LED_PC13();
            m_stage = M_STAGE_D_TURN;
            total_distance = 0;
            MPU6050_ResetYaw(&g_mpu_data); // 重置Yaw偏航角用于转弯积分
        }
        else if (m_stage == M_STAGE_D_TURN) 
        {
            // D -> A 是弯道，检测偏航角是否达到180度
            if (g_mpu_data.yaw >= 180.0f || g_mpu_data.yaw <= -180.0f)
            {
                SYN6658_Speak("\xBB\xD8\xB5\xBD\x41\xB5\xE3"); // 回到A点停车
                Blink_LED_PC13();
                Motor_Stop(); 
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
