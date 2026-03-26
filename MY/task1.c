#include "stm32f4xx.h"
extern void LineFollow_RunByRawIR(void);
extern IR_DataTypeDef g_ir_data;
extern MPU6050_DataTypeDef g_mpu_data;
extern StateMachine_TypeDef g_state_machine;


#include "delay.h"
#include "state_machine.h"
#include "pid.h"
#include "motor.h"
#include "mpu6050.h"
#include "infrared.h"
#include "syn6658.h"

// 脉冲数/100cm 比例。用户说100cm测出来2591个脉冲。
#define PULSES_PER_100CM  2591.0f

typedef enum {
    M_STAGE_INIT = 0,
    M_STAGE_A_TO_B,
    M_STAGE_B_TO_C,  // U型弯道 B->C
    M_STAGE_C_TO_D,
    M_STAGE_D_TO_A,  // U型弯道 D->A
    M_STAGE_DONE
} MoveStage_t;

static MoveStage_t s_move_stage = M_STAGE_INIT;
static float s_start_yaw = 0.0f;

/* 计算两个角度的绝对差值，处理跨越区 (-180, 180) */
static float get_angle_diff(float angle1, float angle2) {
    float diff = angle1 - angle2;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

void Blink_LED_PC13(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 初始化PC13为输出
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 亮灯
    Delay_ms(200);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);   // 灭灯
}

void Task1_Run(void)
{
    s_move_stage = M_STAGE_INIT;
    SM_Init(&g_state_machine, 3); // 任意

    Motor_Enable(1);
    IR_ResetTracking();
    s_start_yaw = g_mpu_data.yaw;

    // 起点A播报: "起点A点"
    SYN6658_Speak("\xC6\xF0\xB5\xE3\x41\xB5\xE3"); 
    Blink_LED_PC13();
    
    s_move_stage = M_STAGE_A_TO_B;

    while (1)
    {
        IR_Read(&g_ir_data);
        MPU6050_ReadAll(&g_mpu_data);

        // 如果在弯道段，则不再使用节点全黑来判断转移，而是依赖偏航角转过了约 175 度
        if (s_move_stage == M_STAGE_B_TO_C) {
            float diff = get_angle_diff(g_mpu_data.yaw, s_start_yaw);
            if (diff > 170.0f || diff < -170.0f) {
                // 转过了 180度
                // 播报: "到达C点"
                SYN6658_Speak("\xB5\xBD\xB4\xEF\x43\xB5\xE3"); 
                Blink_LED_PC13();
                s_move_stage = M_STAGE_C_TO_D;
                // 让它稍微再走一点点避免立即退出黑线
            }
        }
        else if (s_move_stage == M_STAGE_D_TO_A) {
            float diff = get_angle_diff(g_mpu_data.yaw, s_start_yaw);
            if (diff > 170.0f || diff < -170.0f) {
                // 转过了 180度
                // 播报: "回到A点"
                SYN6658_Speak("\xBB\xD8\xB5\xBD\x41\xB5\xE3"); 
                Blink_LED_PC13();
                s_move_stage = M_STAGE_DONE;
            }
        }
        else
        {
            // 在直线段，检查路口黑线
            uint8_t evt = 0;
            if (g_ir_data.S1 == 1 && g_ir_data.S2 == 1 && g_ir_data.S3 == 1 && g_ir_data.S4 == 1 && g_ir_data.S5 == 1) {
                if (g_ir_data.tracking_mode != 2) {
                    g_ir_data.tracking_mode = 2; // cross
                    evt = 99;
                }
            } else {
                g_ir_data.tracking_mode = 1;
            }

            if (evt == 99) {
                if (s_move_stage == M_STAGE_A_TO_B) {
                    // 播报: "到达B点"
                    SYN6658_Speak("\xB5\xBD\xB4\xEF\x42\xB5\xE3"); 
                    Blink_LED_PC13();
                    s_move_stage = M_STAGE_B_TO_C;
                    s_start_yaw = g_mpu_data.yaw; // 记录开始转弯的角度
                } else if (s_move_stage == M_STAGE_C_TO_D) {
                    // 播报: "到达D点"
                    SYN6658_Speak("\xB5\xBD\xB4\xEF\x44\xB5\xE3"); 
                    Blink_LED_PC13();
                    s_move_stage = M_STAGE_D_TO_A;
                    s_start_yaw = g_mpu_data.yaw; // 记录开始转弯的角度
                }
            }
        }

        if (s_move_stage == M_STAGE_DONE) {
            Motor_Stop();
            break;
        }

        // 统一使用红外巡线行走
        LineFollow_RunByRawIR();

        Delay_ms(10);
    }
}
void Task2_Run(void) {}
void Task3_Run(void) {}
void Task4_Run(void) {}
