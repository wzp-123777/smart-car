/**
 * @file    state_machine.c
 * @brief   小车状态机实现
 * @note    当前主工程并未把本状态机作为唯一运行入口。
 *          OpenMV 现为常在线主动上报模式，因此这里不再发送视觉触发命令。
 */

#include "state_machine.h"
#include "motor.h"
#include "pid.h"
#include "infrared.h"
#include "openmv.h"
#include "syn6658.h"
#include "mpu6050.h"
#include "alert.h"

/* ==================== 外部变量（在main.c中定义） ==================== */
extern uint32_t HAL_GetTick(void);
extern void delay_ms(__IO uint32_t nTime);

/**
 * @brief  初始化状态机
 */
void SM_Init(StateMachine_TypeDef *sm, uint8_t total_points)
{
    sm->current_state     = STATE_IDLE;
    sm->last_state        = STATE_IDLE;
    sm->checkpoint_count  = 0;
    sm->total_checkpoints = total_points;
    sm->state_enter_tick  = HAL_GetTick();
    sm->timeout_ms        = 0;
    sm->detected_obj_id   = 0;
}

/**
 * @brief  切换到新状态
 */
void SM_TransitionTo(StateMachine_TypeDef *sm, CarState_TypeDef new_state)
{
    sm->last_state       = sm->current_state;
    sm->current_state    = new_state;
    sm->state_enter_tick = HAL_GetTick();

    switch (new_state)
    {
    case STATE_STOP_AND_DETECT:
        sm->timeout_ms = 500;
        break;
    case STATE_VISION_DETECT:
        sm->timeout_ms = 3000;
        break;
    case STATE_VOICE_REPORT:
        sm->timeout_ms = 4000;
        break;
    case STATE_TURN_LEFT:
    case STATE_TURN_RIGHT:
        sm->timeout_ms = 2000;
        break;
    case STATE_U_TURN:
        sm->timeout_ms = 4000;
        break;
    default:
        sm->timeout_ms = 0;
        break;
    }
}

/**
 * @brief  检查超时
 */
uint8_t SM_IsTimeout(StateMachine_TypeDef *sm)
{
    if (sm->timeout_ms == 0)
    {
        return 0;
    }
    return (HAL_GetTick() - sm->state_enter_tick >= sm->timeout_ms) ? 1 : 0;
}

/**
 * @brief  状态机核心处理函数
 * @note   当前文件保留为扩展框架，不是 main.c 唯一执行主线。
 */
void SM_Process(StateMachine_TypeDef *sm, CarEvent_TypeDef event)
{
    switch (sm->current_state)
    {
    case STATE_IDLE:
        Motor_Stop();
        if (event == EVENT_START)
        {
            sm->checkpoint_count = 0;
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    case STATE_LINE_FOLLOW:
        if (event == EVENT_CROSS_DETECTED)
        {
            sm->checkpoint_count++;
            Alert_Checkpoint(100);
            /* OpenMV 常在线主动上报，这里不再发送触发命令。 */
        }
        else if (event == EVENT_REACHED_END)
        {
            Motor_Stop();
            SM_TransitionTo(sm, STATE_FINISHED);
        }

        if (g_openmv_data.is_new)
        {
            OpenMV_DataTypeDef result = OpenMV_GetResult();
            sm->detected_obj_id = result.object_id;
            OpenMV_ClearNewFlag();
            SYN6658_ReportPoint(sm->checkpoint_count);
            SYN6658_ReportObject(sm->detected_obj_id);
        }
        break;

    case STATE_TURN_LEFT:
        Motor_SetLeft(-300);
        Motor_SetRight(300);
        if (event == EVENT_TURN_DONE || SM_IsTimeout(sm))
        {
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    case STATE_TURN_RIGHT:
        Motor_SetLeft(300);
        Motor_SetRight(-300);
        if (event == EVENT_TURN_DONE || SM_IsTimeout(sm))
        {
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    case STATE_U_TURN:
        Motor_SetLeft(-400);
        Motor_SetRight(400);
        if (event == EVENT_TURN_DONE || SM_IsTimeout(sm))
        {
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    case STATE_FINISHED:
        Motor_Stop();
        Alert_Error(3);
        Alert_LED_On();
        break;

    case STATE_ERROR:
        Motor_Stop();
        Alert_Error(5);
        break;

    default:
        SM_TransitionTo(sm, STATE_ERROR);
        break;
    }
}

/**
 * @brief  获取状态名称（调试打印用）
 */
const char* SM_GetStateName(CarState_TypeDef state)
{
    switch (state)
    {
    case STATE_IDLE:            return "IDLE";
    case STATE_LINE_FOLLOW:     return "LINE_FOLLOW";
    case STATE_STOP_AND_DETECT: return "STOP_DETECT";
    case STATE_VISION_DETECT:   return "VISION";
    case STATE_VOICE_REPORT:    return "VOICE";
    case STATE_TURN_LEFT:       return "TURN_L";
    case STATE_TURN_RIGHT:      return "TURN_R";
    case STATE_U_TURN:          return "U_TURN";
    case STATE_FINISHED:        return "FINISHED";
    case STATE_ERROR:           return "ERROR";
    default:                    return "UNKNOWN";
    }
}
