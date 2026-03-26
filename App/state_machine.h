/**
 * @file    state_machine.h
 * @brief   小车状态机框架
 * @note    定义状态机框架与扩展状态。
 *          当前主工程主要由 main.c + task1.c 调度，本状态机保留给后续任务扩展。
 */
#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "stm32f4xx.h"

/* ==================== 状态枚举 ==================== */
typedef enum {
    STATE_IDLE = 0,         /* 空闲/等待启动 */
    STATE_LINE_FOLLOW,      /* 巡线模式（正常行驶） */
    STATE_STOP_AND_DETECT,  /* 预留状态：停车检测 */
    STATE_VISION_DETECT,    /* 预留状态：视觉识别 */
    STATE_VOICE_REPORT,     /* 预留状态：语音播报 */
    STATE_TURN_LEFT,        /* 左转弯 */
    STATE_TURN_RIGHT,       /* 右转弯 */
    STATE_U_TURN,           /* 掉头 */
    STATE_FINISHED,         /* 任务完成 */
    STATE_ERROR,            /* 错误状态 */
} CarState_TypeDef;

/* ==================== 事件枚举 ==================== */
typedef enum {
    EVENT_NONE = 0,         /* 无事件 */
    EVENT_START,            /* 启动按键按下 */
    EVENT_CROSS_DETECTED,   /* 检测到十字路口/标记线 */
    EVENT_LINE_LOST,        /* 脱线 */
    EVENT_VISION_DONE,      /* OpenMV识别完成 */
    EVENT_VOICE_DONE,       /* 语音播报完成 */
    EVENT_TURN_DONE,        /* 转弯完成 */
    EVENT_REACHED_END,      /* 到达终点 */
    EVENT_TIMEOUT,          /* 超时 */
} CarEvent_TypeDef;

/* ==================== 状态机上下文 ==================== */
typedef struct {
    CarState_TypeDef  current_state;    /* 当前状态 */
    CarState_TypeDef  last_state;       /* 上一个状态（用于调试） */
    uint8_t           checkpoint_count; /* 已经过的巡检点数量 */
    uint8_t           total_checkpoints;/* 总巡检点数量 */
    uint32_t          state_enter_tick; /* 进入当前状态的时间戳 */
    uint32_t          timeout_ms;       /* 当前状态超时时间 */
    uint8_t           detected_obj_id;  /* 最近一次识别的物体ID */
} StateMachine_TypeDef;

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化状态机
 * @param  sm: 状态机上下文指针
 * @param  total_points: 总巡检点数量
 */
void SM_Init(StateMachine_TypeDef *sm, uint8_t total_points);

/**
 * @brief  状态机主循环处理（在main while(1)中调用）
 * @param  sm: 状态机上下文
 * @param  event: 当前事件
 */
void SM_Process(StateMachine_TypeDef *sm, CarEvent_TypeDef event);

/**
 * @brief  切换状态
 * @param  sm: 状态机上下文
 * @param  new_state: 新状态
 */
void SM_TransitionTo(StateMachine_TypeDef *sm, CarState_TypeDef new_state);

/**
 * @brief  检查当前状态是否超时
 * @param  sm: 状态机上下文
 * @retval 1=超时, 0=未超时
 */
uint8_t SM_IsTimeout(StateMachine_TypeDef *sm);

/**
 * @brief  获取当前状态名称字符串（调试用）
 */
const char* SM_GetStateName(CarState_TypeDef state);

#endif /* __STATE_MACHINE_H */
