import re
import sys

def rewrite(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # 1. Tracker logic replacement
    tracker_replacement = r'''static float track_kp = 2.0f;   /* 外环(位置环) 比例系数 - 依新精度微调 */
static float track_kd = 0.5f;   /* 外环(位置环) 微分系数 - 依新精度微调 */
static float track_last_error = 0.0f;
#define BASE_TARGET_SPEED 30.0f /* 基础目标速度（脉冲/10ms），可根据实际情况调节 */

static void LineFollow_RunByRawIR(void)
{
    float error = 0.0f;
    float p_out = 0.0f;
    float d_out = 0.0f;
    float total_adjust = 0.0f;
    float speed_left = BASE_TARGET_SPEED;
    float speed_right = BASE_TARGET_SPEED;
    uint8_t hit_count = 0;

    /* 
     * 传感器分布: [0]最左, [1]偏左, [2]中间, [3]偏右, [4]最右 
     * 根据要求：五路循迹精度放到10
     */
    if (g_ir_data.sensor[0]) { error -= 10.0f; hit_count++; }
    if (g_ir_data.sensor[1]) { error -= 5.0f;  hit_count++; }
    if (g_ir_data.sensor[2]) { error += 0.0f;  hit_count++; }
    if (g_ir_data.sensor[3]) { error += 5.0f;  hit_count++; }
    if (g_ir_data.sensor[4]) { error += 10.0f; hit_count++; }
    
    if (hit_count > 0) {
        error = error / (float)hit_count; /* 计算平均误差中心 */
    } else {
        /* 完全丢失黑线时，使用最后一次记录的误差极性放大 */
        if (track_last_error < 0.0f) { error = -12.0f; }
        else if (track_last_error > 0.0f) { error = 12.0f; }
        else { error = 0.0f; }
    }

    /* 外环：位置 PID 计算 */
    p_out = track_kp * error;
    d_out = track_kd * (error - track_last_error);
    total_adjust = p_out + d_out;
    
    /* 
     * 控制手段: 串级PID分配目标速度（允许分配到负值反相）
     * 策略：不加大外侧轮速度，只减小（或反向）内侧轮，以产生大差速转向。
     */
    if (total_adjust < 0.0f)
    {
        /* 偏左需左转：减小左侧内轮速度 */
        speed_left = BASE_TARGET_SPEED + total_adjust; // total_adjust为负，相加即减速/反转
        speed_right = BASE_TARGET_SPEED;
        g_line_debug_mode = "PID_L";
    }
    else if (total_adjust > 0.0f)
    {
        /* 偏右需右转：减小右侧内轮速度 */
        speed_left = BASE_TARGET_SPEED;
        speed_right = BASE_TARGET_SPEED - total_adjust;
        g_line_debug_mode = "PID_R";
    }
    else
    {
        speed_left = BASE_TARGET_SPEED;
        speed_right = BASE_TARGET_SPEED;
        g_line_debug_mode = "PID_C";
    }
    
    /* 更新上一拍误差 */
    if (hit_count > 0) track_last_error = error;

    /* 将计算出的分配速度送到内环（速度环）目标 */
    g_pid_left.target = speed_left;
    g_pid_right.target = speed_right;
}'''

    content = re.sub(r'static float track_kp = 60\.0f;.*?\n\}\n', tracker_replacement + '\n', content, flags=re.DOTALL)

    # 2. Timer Interrupt Replacement
    tim_replacement = r'''void TIM6_DAC_IRQHandler(void)
{
    float pwm_left = 0.0f;
    float pwm_right = 0.0f;

    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        
        /* ===== 1. 获取编码器反馈值 ===== */
        g_encoder_left  = Encoder_Read_TIM2(); 
        g_encoder_right = Encoder_Read_TIM3();
        
        /* ===== 2. MPU6050 姿态结算 ===== */
        MPU6050_UpdateYaw(&g_mpu_data, 0.01f);

        /* ===== 4. 内环：PID速度闭环执行 ===== */
        pwm_left = PID_Incremental(&g_pid_left, g_pid_left.target, (float)g_encoder_left);
        pwm_right = PID_Incremental(&g_pid_right, g_pid_right.target, (float)g_encoder_right);
        
        /* 驱动电机输出PWM */
        Motor_SetLeft((int16_t)pwm_left);
        Motor_SetRight((int16_t)pwm_right);

        /* 发送事件标志给主循环 */
        g_flag_10ms = 1;
    }
}'''
    content = re.sub(r'void TIM6_DAC_IRQHandler\(void\).*?\}\r?\n\}', tim_replacement + '\n', content, flags=re.DOTALL)

    # 3. Main Init Replacement
    match_main = re.search(r'int main\(void\).*?Motor_GPIO_Init\(\);.*?MX_USART1_Init\(\);', content, re.DOTALL)
    if not match_main:
        print("COULD NOT FIND MAIN BLOCK in", filepath)
        return False

    old_main_block = match_main.group(0)
    
    # We only want to replace the specific encoder/PWM init lines inside main
    init_new = r'''MX_TIM1_PWM_Init();           // 定时器1 PWM (PA8~PA11)
Motor_GPIO_Init();            // 电机引脚/接口

    /* 闭环必须：使能编码器与定时器 */
    MX_TIM2_Encoder_Init();
    MX_TIM3_Encoder_Init();
    MX_TIM6_Init();               // 核心计算定时器 (10ms)
    
    /* 速度环PID参数初始化 (Kp, Ki, Kd, OutMax, OutMin) */
    /* N20电机经验值：Kp=15.0, Ki=1.5, Kd=0.5; PWM全量程给1000或500对应最大值 */
    PID_Init(&g_pid_left, 15.0f, 1.5f, 0.5f, 500.0f, -500.0f);
    PID_Init(&g_pid_right, 15.0f, 1.5f, 0.5f, 500.0f, -500.0f);

    MX_USART1_Init();'''
    
    new_main_block = re.sub(r'MX_TIM1_PWM_Init.*?MX_USART1_Init\(\);', init_new, old_main_block, flags=re.DOTALL)
    
    content = content.replace(old_main_block, new_main_block)

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

    return True

if rewrite(r'd:\KeilMDKARM5.35\smart-car\App\main.c'):
    rewrite(r'd:\KeilMDKARM5.35\smart-car\MY\main.c')
    print('Safety cascade PID applied!')
