import sys

file_path = r'd:\KeilMDKARM5.35\smart-car\App\main.c'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

start_str = 'void LineFollow_RunByRawIR(void)\n{'
end_str = '}\n/* ================================================================'

new_func = '''void LineFollow_RunByRawIR(void)
{
    float base_speed = LINE_FOLLOW_TARGET_STRAIGHT;
    float gyro_rate = g_mpu_data.gyro_z_dps - g_mpu_gyro_z_bias;
    float speed_left = LINE_FOLLOW_TARGET_STRAIGHT;
    float speed_right = LINE_FOLLOW_TARGET_STRAIGHT;
    
    uint8_t left_edge_on = g_ir_data.sensor[0];  // 1最左
    uint8_t left_near_on = g_ir_data.sensor[1];  // 2偏左
    uint8_t center_on    = g_ir_data.sensor[2];  // 3中间
    uint8_t right_near_on= g_ir_data.sensor[3];  // 4偏右
    uint8_t right_edge_on= g_ir_data.sensor[4];  // 5最右

    g_line_debug_left_score = (left_edge_on ? 2U : 0U) + (left_near_on ? 1U : 0U);
    g_line_debug_right_score = (right_edge_on ? 2U : 0U) + (right_near_on ? 1U : 0U);

    // 状态记录: 0=直行/中间, 1=曾在偏左(2), 2=曾在偏右(4)
    static uint8_t turn_history = 0; 
    
    // 中间触发并且两侧都没有触发，重置状态
    if (center_on && !left_near_on && !right_near_on && !left_edge_on && !right_edge_on) {
        turn_history = 0;
    }

    // 根据陀螺仪动态计算差速比 [2.0, 4.0]
    float gyro_abs = gyro_rate > 0 ? gyro_rate : -gyro_rate;
    // 假设偏航角速度通常在 0~60 左右
    float diff_ratio = 2.0f + (gyro_abs * 0.04f); 
    if (diff_ratio > 4.0f) diff_ratio = 4.0f;
    if (diff_ratio < 2.0f) diff_ratio = 2.0f;

    if (g_ir_data.all_white != 0U)
    {
        // 全白丢失
        speed_left = 0.0f;
        speed_right = 0.0f;
        g_line_debug_mode = "LOST";
    }
    else if (g_ir_data.all_black != 0U)
    {
        // 全黑十字，直接冲或者作为直行
        turn_history = 0;
        speed_left = base_speed;
        speed_right = base_speed;
        g_line_debug_mode = "BLACK";
    }
    else
    {
        // 核心循迹逻辑
        if (left_edge_on && turn_history == 1) 
        {
            // 当 1-5 (1) 在经过了 2-4 (2) 变化的时候才能选择内轮抱死
            speed_left = 0.0f;           // 左内轮抱死
            speed_right = base_speed;
            g_line_debug_mode = "LOCK_L";
        }
        else if (right_edge_on && turn_history == 2)
        {
            // 当 1-5 (5) 在经过了 2-4 (4) 变化的时候才能选择内轮抱死
            speed_left = base_speed;
            speed_right = 0.0f;          // 右内轮抱死
            g_line_debug_mode = "LOCK_R";
        }
        else if (left_near_on)
        {
            // 2路 变化，记录历史，同向差速
            turn_history = 1;
            speed_left = base_speed / diff_ratio;
            speed_right = base_speed;
            g_line_debug_mode = "DIFF_L";
        }
        else if (right_near_on)
        {
            // 4路 变化，记录历史，同向差速
            turn_history = 2;
            speed_left = base_speed;
            speed_right = base_speed / diff_ratio;
            g_line_debug_mode = "DIFF_R";
        }
        else if (left_edge_on) 
        {
            // 若没有经过2直接触发了1(交叉路口干扰)，只给轻微/常规同向差速
            speed_left = base_speed / diff_ratio;
            speed_right = base_speed;
            g_line_debug_mode = "JMP_L";
        }
        else if (right_edge_on)
        {
            // 若没有经过4直接触发了5，只给轻微/常规同向差速
            speed_left = base_speed;
            speed_right = base_speed / diff_ratio;
            g_line_debug_mode = "JMP_R";
        }
        else 
        {
            // 默认中心直行
            speed_left = base_speed;
            speed_right = base_speed;
            g_line_debug_mode = "CENTER";
        }
    }

    g_pid_left.target = speed_left;
    g_pid_right.target = speed_right;
    g_line_debug_left_pwm = (int16_t)speed_left;
    g_line_debug_right_pwm = (int16_t)speed_right;
}
/* ================================================================'''

start_idx = content.find(start_str)
end_idx = content.find(end_str)

if start_idx != -1 and end_idx != -1:
    new_content = content[:start_idx] + new_func + content[end_idx + len(end_str):]
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print("Replace successful")
else:
    print("Could not find boundaries")
    print(start_idx, end_idx)
