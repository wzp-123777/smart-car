import os

file_path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\task1.c'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# I will recreate the task1 logic manually as asked by user.
# The user asked:
# Instead of M_STAGE_B_TURN and D_TURN, we use 180 degree MPU diff to detect semi-circle curve tracking

new_task1 = '''
#include "main.h"
#include "state_machine.h"
#include "pid.h"
#include "motor.h"
#include "mpu6050.h"
#include "infrared.h"

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

void Task1_Run(void)
{
    s_move_stage = M_STAGE_INIT;
    SM_Init(&g_state_machine, 3); // 任意
    
    Motor_Enable(1);
    IR_ResetTracking();
    s_start_yaw = g_mpu_data.yaw;

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
                s_move_stage = M_STAGE_C_TO_D;
                // 让它稍微再走一点点避免立即退出黑线
            }
        } 
        else if (s_move_stage == M_STAGE_D_TO_A) {
            float diff = get_angle_diff(g_mpu_data.yaw, s_start_yaw);
            if (diff > 170.0f || diff < -170.0f) {
                // 转过了 180度
                s_move_stage = M_STAGE_DONE;
            }
        }
        else 
        {
            // 在直线段，检查路口黑线
            CarEvent_TypeDef evt = EVENT_NONE;
            if (g_ir_data.S1 == 1 && g_ir_data.S2 == 1 && g_ir_data.S3 == 1 && g_ir_data.S4 == 1 && g_ir_data.S5 == 1) {
                if (g_ir_data.tracking_mode != 2) {
                    g_ir_data.tracking_mode = 2; // cross
                    evt = EVENT_CROSSROAD;
                }
            } else {
                g_ir_data.tracking_mode = 1; 
            }

            if (evt == EVENT_CROSSROAD) {
                if (s_move_stage == M_STAGE_A_TO_B) {
                    s_move_stage = M_STAGE_B_TO_C;
                    s_start_yaw = g_mpu_data.yaw; // 记录开始转弯的角度
                } else if (s_move_stage == M_STAGE_C_TO_D) {
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

        delay_ms(10);
    }
}
'''
with open(file_path, 'w', encoding='utf-8') as f:
    f.write(new_task1)

print('Task1 updated!')
