/**
 * @file    state_machine.c
 * @brief   小车状态机实现
 * @note    核心逻辑：根据当前状态+事件决定下一步动作
 *          比赛时主要修改 SM_Process 中的状态转换条件
 */

#include "state_machine.h"
#include "motor.h"
#include "pid.h"
#include "infrared.h"
#include "openmv.h"
#include "syn6658.h"
#include "mpu6050.h"

/* ==================== 外部变量（在main.c中定义） ==================== */
extern TIM_HandleTypeDef htim1;     /* PWM定时器 */
extern UART_HandleTypeDef huart1;   /* OpenMV串口 */
extern UART_HandleTypeDef huart2;   /* SYN6658串口 */

/**
 * @brief  初始化状态机
 */
void SM_Init(StateMachine_TypeDef *sm, uint8_t total_points)
{
    sm->current_state    = STATE_IDLE;
    sm->last_state       = STATE_IDLE;
    sm->checkpoint_count = 0;
    sm->total_checkpoints = total_points;
    sm->state_enter_tick = HAL_GetTick();
    sm->timeout_ms       = 0;
    sm->detected_obj_id  = 0;
}

/**
 * @brief  切换到新状态
 */
void SM_TransitionTo(StateMachine_TypeDef *sm, CarState_TypeDef new_state)
{
    sm->last_state       = sm->current_state;
    sm->current_state    = new_state;
    sm->state_enter_tick = HAL_GetTick();

    /* 根据不同状态设置超时时间 */
    switch (new_state)
    {
    case STATE_STOP_AND_DETECT:
        sm->timeout_ms = 500;   /* 停车稳定500ms */
        break;
    case STATE_VISION_DETECT:
        sm->timeout_ms = 3000;  /* 视觉识别最多等3秒 */
        break;
    case STATE_VOICE_REPORT:
        sm->timeout_ms = 4000;  /* 语音播报最多4秒 */
        break;
    case STATE_TURN_LEFT:
    case STATE_TURN_RIGHT:
        sm->timeout_ms = 2000;  /* 转弯最多2秒 */
        break;
    case STATE_U_TURN:
        sm->timeout_ms = 4000;  /* 掉头最多4秒 */
        break;
    default:
        sm->timeout_ms = 0;     /* 无超时 */
        break;
    }
}

/**
 * @brief  检查超时
 */
uint8_t SM_IsTimeout(StateMachine_TypeDef *sm)
{
    if (sm->timeout_ms == 0) return 0;
    return (HAL_GetTick() - sm->state_enter_tick >= sm->timeout_ms) ? 1 : 0;
}

/**
 * @brief  状态机核心处理函数
 * @note   在 main.c 的 while(1) 中以 ~10ms 周期调用
 * 
 * 【比赛时修改要点】
 *  1. STATE_STOP_AND_DETECT: 修改停车条件（几路传感器全黑=到达标记）
 *  2. STATE_VISION_DETECT: 修改识别完成后的动作
 *  3. 添加/修改转弯逻辑
 */
void SM_Process(StateMachine_TypeDef *sm, CarEvent_TypeDef event)
{
    switch (sm->current_state)
    {
    /* ==================== 空闲状态 ==================== */
    case STATE_IDLE:
        /* 电机停止，等待启动信号 */
        Motor_Stop(&htim1);

        if (event == EVENT_START)
        {
            sm->checkpoint_count = 0;
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    /* ==================== 巡线模式 ==================== */
    case STATE_LINE_FOLLOW:
        /*
         * 巡线控制在定时器中断中执行（10ms周期）
         * 这里只处理状态切换事件
         */
        if (event == EVENT_CROSS_DETECTED)
        {
            /* 检测到十字路口/标记线 → 停车检测 */
            sm->checkpoint_count++;
            Motor_Stop(&htim1);
            SM_TransitionTo(sm, STATE_STOP_AND_DETECT);
        }
        else if (event == EVENT_REACHED_END)
        {
            /* 到达终点 */
            Motor_Stop(&htim1);
            SM_TransitionTo(sm, STATE_FINISHED);
        }
        break;

    /* ==================== 停车稳定 ==================== */
    case STATE_STOP_AND_DETECT:
        Motor_Stop(&htim1);

        /* 等待500ms稳定后进入视觉识别 */
        if (SM_IsTimeout(sm))
        {
            /* 发送识别指令给OpenMV */
            OpenMV_SendCmd(&huart1, OPENMV_CMD_DETECT);
            SM_TransitionTo(sm, STATE_VISION_DETECT);
        }
        break;

    /* ==================== 视觉识别 ==================== */
    case STATE_VISION_DETECT:
        Motor_Stop(&htim1);

        if (event == EVENT_VISION_DONE)
        {
            /* OpenMV返回了识别结果 */
            OpenMV_DataTypeDef result = OpenMV_GetResult();
            sm->detected_obj_id = result.object_id;
            OpenMV_ClearNewFlag();

            /* 进入语音播报 */
            SYN6658_ReportPoint(&huart2, sm->checkpoint_count);
            HAL_Delay(1500);  /* 等待"到达第X个巡检点"播完 */
            SYN6658_ReportObject(&huart2, sm->detected_obj_id);

            SM_TransitionTo(sm, STATE_VOICE_REPORT);
        }
        else if (SM_IsTimeout(sm))
        {
            /* 超时未识别到，播报"未识别"后继续巡线 */
            SYN6658_Speak(&huart2, "未识别到目标");
            SM_TransitionTo(sm, STATE_VOICE_REPORT);
        }
        break;

    /* ==================== 语音播报 ==================== */
    case STATE_VOICE_REPORT:
        Motor_Stop(&htim1);

        if (event == EVENT_VOICE_DONE || SM_IsTimeout(sm))
        {
            /* 播报完成，判断是否已到达所有巡检点 */
            if (sm->checkpoint_count >= sm->total_checkpoints)
            {
                SM_TransitionTo(sm, STATE_FINISHED);
            }
            else
            {
                /* 继续巡线 */
                SM_TransitionTo(sm, STATE_LINE_FOLLOW);
            }
        }
        break;

    /* ==================== 左转 ==================== */
    case STATE_TURN_LEFT:
        Motor_SetLeft(&htim1, -300);   /* 左轮反转 */
        Motor_SetRight(&htim1, 300);   /* 右轮正转 */

        if (event == EVENT_TURN_DONE || SM_IsTimeout(sm))
        {
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    /* ==================== 右转 ==================== */
    case STATE_TURN_RIGHT:
        Motor_SetLeft(&htim1, 300);
        Motor_SetRight(&htim1, -300);

        if (event == EVENT_TURN_DONE || SM_IsTimeout(sm))
        {
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    /* ==================== 掉头 ==================== */
    case STATE_U_TURN:
        Motor_SetLeft(&htim1, -400);
        Motor_SetRight(&htim1, 400);

        if (event == EVENT_TURN_DONE || SM_IsTimeout(sm))
        {
            SM_TransitionTo(sm, STATE_LINE_FOLLOW);
        }
        break;

    /* ==================== 任务完成 ==================== */
    case STATE_FINISHED:
        Motor_Stop(&htim1);
        /* 可以加LED指示或蜂鸣器提示 */
        break;

    /* ==================== 错误状态 ==================== */
    case STATE_ERROR:
        Motor_Stop(&htim1);
        /* 错误处理：闪烁LED等 */
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
