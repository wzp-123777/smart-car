import os

def process_pid_h(path):
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    if 'last_derivative' not in content:
        content = content.replace('float integral_max; // 积分限幅（防饱和）', 'float integral_max; // 积分限幅（防饱和）\n\n    float last_derivative; // 上次微分滤波结果')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(content)

def process_pid_c(path):
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    if 'last_derivative' not in content:
        content = content.replace('pid->integral_max = out_max * 0.6f;  // 积分限幅为输出的60%', 'pid->integral_max = out_max * 0.6f;  // 积分限幅为输出的60%\n\n    pid->last_derivative = 0;')
        content = content.replace(
'''    /* 增量式PID核心公式 */
    delta = pid->Kp * (pid->err - pid->err_last)
          + pid->Ki * pid->err
          + pid->Kd * (pid->err - 2.0f * pid->err_last + pid->err_prev);''', 
'''    /* D部分低通滤波 (一阶低通滤波) */
    float D_raw_inc = (pid->err - 2.0f * pid->err_last + pid->err_prev);
    // 滤波系数 alpha 取 0.3 (0~1之间, 越小滤波越强)
    pid->last_derivative = 0.3f * D_raw_inc + 0.7f * pid->last_derivative;

    /* 增量式PID核心公式 */
    delta = pid->Kp * (pid->err - pid->err_last)
          + pid->Ki * pid->err
          + pid->Kd * pid->last_derivative;''')
        content = content.replace(
'''    /* 位置式PID */
    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->integral
                + pid->Kd * (pid->err - pid->err_last);''',
'''    /* D部分低通滤波 */
    float D_raw_pos = (pid->err - pid->err_last);
    pid->last_derivative = 0.3f * D_raw_pos + 0.7f * pid->last_derivative;

    /* 位置式PID */
    pid->output = pid->Kp * pid->err
                + pid->Ki * pid->integral
                + pid->Kd * pid->last_derivative;''')
        content = content.replace('pid->integral = 0;', 'pid->integral = 0;\n    pid->last_derivative = 0;')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(content)

process_pid_h('App/pid.h')
process_pid_h('MY/pid.h')
process_pid_c('App/pid.c')
process_pid_c('MY/pid.c')
