/**
 * @file    pid.c
 * @brief   增量式PID控制器实现
 * @note    核心算法：增量式 PID
 *          公式：delta = Kp*(err-err_last) + Ki*err + Kd*(err-2*err_last+err_prev)
 *          output += delta
 * 
 * 【PID调参指南】
 *  1. 先将 Ki=0, Kd=0，只调 Kp：
 *     - 从小到大增加 Kp，直到电机能基本跟上目标速度
 *     - 如果出现明显震荡，适当减小 Kp
 *  2. 然后加入 Ki（从 0.01 开始）：
 *     - Ki 消除稳态误差，让速度精确到达目标值
 *     - Ki 过大会导致超调和震荡
 *  3. 最后微调 Kd（可选，从 0.1 开始）：
 *     - Kd 抑制超调，提高响应速度
 *     - Kd 过大会放大噪声，引起抖动
 *  4. 典型参数范围（N20电机 + 10ms周期）：
 *     - Kp: 5.0 ~ 20.0
 *     - Ki: 0.5 ~ 3.0
 *     - Kd: 0.0 ~ 2.0
 */

#include "pid.h"

/**
 * @brief  初始化PID参数
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float out_max, float out_min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target = 0;
    pid->actual = 0;

    pid->err      = 0;
    pid->err_last = 0;
    pid->err_prev = 0;

    pid->output     = 0;
    pid->output_max = out_max;
    pid->output_min = out_min;

    pid->integral     = 0;
    pid->integral_max = out_max * 0.6f;  // 积分限幅为输出的60%
}

/**
 * @brief  增量式PID计算（推荐）
 * @note   每10ms在定时器中断中调用一次
 *         增量式不会积分饱和，切换模式时更安全
 */
float PID_Incremental(PID_TypeDef *pid, float target, float actual)
{
    float delta;

    pid->target = target;
    pid->actual = actual;

    /* 计算当前误差 */
    pid->err = target - actual;

    /* 增量式PID核心公式 */
    delta = pid->Kp * (pid->err - pid->err_last)
          + pid->Ki * pid->err
          + pid->Kd * (pid->err - 2.0f * pid->err_last + pid->err_prev);

    /* 累加到输出 */
    pid->output += delta;

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    /* 更新历史误差 */
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;

    return pid->output;
}

/**
 * @brief  位置式PID计算（备用）
 * @note   带积分限幅防饱和
 */
float PID_Positional(PID_TypeDef *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;

    pid->err = target - actual;

    /* 积分累加 + 限幅 */
    pid->integral += pid->err;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    /* 位置式PID */
    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->integral
                + pid->Kd * (pid->err - pid->err_last);

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    pid->err_last = pid->err;

    return pid->output;
}

/**
 * @brief  重置PID状态
 * @note   在切换运动模式（巡线→停车等）时调用，避免积分遗留
 */
void PID_Reset(PID_TypeDef *pid)
{
    pid->err      = 0;
    pid->err_last = 0;
    pid->err_prev = 0;
    pid->output   = 0;
    pid->integral = 0;
}
