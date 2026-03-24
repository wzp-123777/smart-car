import sys
import re

with open(r'd:\KeilMDKARM5.35\smart-car\App\main.c', 'r', encoding='utf-8') as f:
    content = f.read()

pattern = r'static void LineFollow_RunByRawIR\(void\)\s*\{.*?\n\}\s*\n'
match = re.search(pattern, content, flags=re.DOTALL)

if not match:
    print('Pattern not found')
    sys.exit(1)

new_func = r'''static float track_kp = 60.0f; /* 比例系数 - 根据实际情况微调 */
static float track_kd = 30.0f; /* 微分系数 - 根据实际情况微调 */
static float track_last_error = 0.0f;

static void LineFollow_RunByRawIR(void)
{
    float error = 0.0f;
    float p_out = 0.0f;
    float d_out = 0.0f;
    float total_adjust = 0.0f;
    int16_t final_left = 300;
    int16_t final_right = 300;
    int16_t inner_speed = 0;
    uint8_t hit_count = 0;

    /* 
     * 传感器分布: [0]最左, [1]偏左, [2]中间, [3]偏右, [4]最右 
     * 压线(黑)时为1。根据位置赋权重计算 error (小车偏向位置)
     */
    if (g_ir_data.sensor[0]) { error -= 4.0f; hit_count++; }
    if (g_ir_data.sensor[1]) { error -= 2.0f; hit_count++; }
    if (g_ir_data.sensor[2]) { error += 0.0f; hit_count++; }
    if (g_ir_data.sensor[3]) { error += 2.0f; hit_count++; }
    if (g_ir_data.sensor[4]) { error += 4.0f; hit_count++; }
    
    if (hit_count > 0)
    {
        error = error / (float)hit_count; /* 计算平均误差中心 */
    }
    else
    {
        /* 完全丢失黑线时，使用最后一次记录的误差极性放大（用于锐角大弯找线） */
        if (track_last_error < 0.0f) { error = -5.0f; }
        else if (track_last_error > 0.0f) { error = 5.0f; }
        else { error = 0.0f; }
    }

    /* PID 计算 */
    p_out = track_kp * error;
    d_out = track_kd * (error - track_last_error);
    total_adjust = p_out + d_out;
    
    /* 
     * 控制策略: 最大限速 300 (即 30%占空比)，坚决不让外侧车轮主动加速！！！
     * 控制手段: 仅对内侧轮进行减速以形成差速转向（内侧快卡死时差速最大）。
     */
    if (total_adjust < 0.0f)
    {
        /* error < 0 代表黑线偏左（车体偏右），需左转，大幅减速左方内侧轮 */
        inner_speed = 300 - (int16_t)(-total_adjust);
        if (inner_speed < 0) { inner_speed = 0; }
        final_left = inner_speed;
        final_right = 300;
        g_line_debug_mode = "PID_L";
    }
    else if (total_adjust > 0.0f)
    {
        /* error > 0 代表黑线偏右（车体偏左），需右转，大幅减速右方内侧轮 */
        inner_speed = 300 - (int16_t)(total_adjust);
        if (inner_speed < 0) { inner_speed = 0; }
        final_left = 300;
        final_right = inner_speed;
        g_line_debug_mode = "PID_R";
    }
    else
    {
        /* 完美居中，均满速30% */
        final_left = 300;
        final_right = 300;
        g_line_debug_mode = "PID_C";
    }
    
    /* 更新上一拍误差，用于下一次微分项计算 (需正常压线才记录有效位置) */
    if (hit_count > 0)
    {
        track_last_error = error;
    }
    
    g_line_debug_left_pwm = final_left;
    g_line_debug_right_pwm = final_right;

    Motor_SetLeft(final_left);
    Motor_SetRight(final_right);
}
'''

new_content = content.replace(match.group(0), new_func + '\n')

with open(r'd:\KeilMDKARM5.35\smart-car\App\main.c', 'w', encoding='utf-8') as f:
    f.write(new_content)

with open(r'd:\KeilMDKARM5.35\smart-car\MY\main.c', 'w', encoding='utf-8') as f:
    f.write(new_content)

print('Success')