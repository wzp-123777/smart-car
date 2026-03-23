/**
 * @file    pid.h
 * @brief   增量式PID控制器 —— 左右轮独立PID
 * @note    适用于电机转速闭环控制，PID参数可在线调节
 */
#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

/* ======================== PID 结构体 ======================== */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数

    float target;       // 目标值（期望转速，单位：编码器脉冲/10ms）
    float actual;       // 实际值（当前转速）

    float err;          // 当前误差
    float err_last;     // 上一次误差
    float err_prev;     // 上上次误差

    float output;       // PID 输出（PWM增量累加后的值）
    float output_max;   // 输出限幅上限
    float output_min;   // 输出限幅下限

    float integral;     // 积分累加值（位置式用，增量式可选）
    float integral_max; // 积分限幅（防饱和）
} PID_TypeDef;

/* ======================== 函数声明 ======================== */

/**
 * @brief  初始化PID参数
 * @param  pid: PID结构体指针
 * @param  kp, ki, kd: PID三个参数
 * @param  out_max, out_min: 输出限幅
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float out_max, float out_min);

/**
 * @brief  增量式PID计算（推荐用于电机调速）
 * @param  pid: PID结构体指针
 * @param  target: 目标转速
 * @param  actual: 实际转速（编码器反馈）
 * @retval PID输出值（PWM占空比）
 */
float PID_Incremental(PID_TypeDef *pid, float target, float actual);

/**
 * @brief  位置式PID计算（备用）
 * @param  pid: PID结构体指针
 * @param  target: 目标值
 * @param  actual: 实际值
 * @retval PID输出值
 */
float PID_Positional(PID_TypeDef *pid, float target, float actual);

/**
 * @brief  重置PID状态（切换模式时调用）
 */
void PID_Reset(PID_TypeDef *pid);

#endif /* __PID_H */
