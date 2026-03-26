import shutil

def rewrite():
    file_path = r"d:\KeilMDKARM5.35\smart-car\App\main.c"
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
        
    start_idx = -1
    for i, l in enumerate(lines):
        if "/* ===== 状态机初始化 ===== */" in l:
            start_idx = i
            break
            
    end_idx = -1
    for i, l in enumerate(lines):
        if i > start_idx and l == "}\n" and lines[i-1] == "    }\n":
            end_idx = i
            break
            
    if start_idx == -1 or end_idx == -1:
        print("未找到边界")
        return
        
    old_task1 = lines[start_idx:end_idx]
    
    # 构建新逻辑
    new_logic = """
/* ==================== 语音播报 GBK 编码 ==================== */
#define TTS_PLEASE_SELECT     "\\xC7\\xEB\\xD1\\xA1\\xD4\\xF1\\xC8\\xCE\\xCE\\xF1" // "请选择任务"
#define TTS_NO_TASK_SELECTED  "\\xCE\\xB4\\xD1\\xA1\\xD4\\xF1\\xC8\\xCE\\xCE\\xF1" // "未选择任务"
#define TTS_START_EXEC        "\\xBF\\xAA\\xCA\\xBC\\xD4\\xCB\\xD0\\xD0"         // "开始执行"

// "任务一" ... "任务四"的GBK
char* TASK_NAMES[5] = {
    "", 
    "\\xC8\\xCE\\xCE\\xF1\\xD2\\xBB", 
    "\\xC8\\xCE\\xCE\\xF1\\xB6\\xFE", 
    "\\xC8\\xCE\\xCE\\xF1\\xC8\\xFD", 
    "\\xC8\\xCE\\xCE\\xF1\\xCB\\xC4"  
};

// "开始任务一" ~ "开始任务四"
char* START_TASK_NAMES[5] = {
    "", 
    "\\xBF\\xAA\\xCA\\xBC\\xC8\\xCE\\xCE\\xF1\\xD2\\xBB", 
    "\\xBF\\xAA\\xCA\\xBC\\xC8\\xCE\\xCE\\xF1\\xB6\\xFE", 
    "\\xBF\\xAA\\xCA\\xBC\\xC8\\xCE\\xCE\\xF1\\xC8\\xFD", 
    "\\xBF\\xAA\\xCA\\xBC\\xC8\\xCE\\xCE\\xF1\\xCB\\xC4"  
};

/* ==================== 系统调度状态 ==================== */
typedef enum {
    SYS_STATE_TASK_SELECT = 0,
    SYS_STATE_TASK_RUNNING     
} SystemState_t;

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,
    KEY_EVENT_LONG
} KeyEvent_t;

SystemState_t g_sys_state = SYS_STATE_TASK_SELECT;
uint8_t g_selected_task = 0; 

KeyEvent_t Get_Key_Event(void) 
{
    static uint32_t key_down_tick = 0;
    static uint8_t  key_pressed = 0;
    
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0) 
    {
        if (key_pressed == 0) 
        {
            delay_ms(20); 
            if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0) 
            {
                key_pressed = 1;
                key_down_tick = HAL_GetTick(); 
            }
        }
    } 
    else 
    {
        if (key_pressed == 1) 
        {
            key_pressed = 0; 
            uint32_t hold_time = HAL_GetTick() - key_down_tick;
            
            if (hold_time >= 1000)      
            {
                return KEY_EVENT_LONG;
            } 
            else if (hold_time >= 50)   
            {
                return KEY_EVENT_SHORT;
            }
        }
    }
    return KEY_EVENT_NONE;
}

void System_Scheduler(void)
{
    SYN6658_Speak(TTS_PLEASE_SELECT); 
    g_sys_state = SYS_STATE_TASK_SELECT;
    g_selected_task = 0;

    while (1)
    {
        if (g_sys_state == SYS_STATE_TASK_SELECT)
        {
            KeyEvent_t key = Get_Key_Event(); 

            if (key == KEY_EVENT_SHORT) 
            {
                g_selected_task++;
                if(g_selected_task > 4) g_selected_task = 1; 
                SYN6658_Speak(TASK_NAMES[g_selected_task]);  
            } 
            else if (key == KEY_EVENT_LONG) 
            {
                if (g_selected_task == 0) 
                {
                    SYN6658_Speak(TTS_NO_TASK_SELECTED); 
                } 
                else 
                {
                    SYN6658_Speak(START_TASK_NAMES[g_selected_task]); 
                    delay_ms(1500); // 等待语音播报结束
                    g_sys_state = SYS_STATE_TASK_RUNNING;
                }
            }
        }
        else if (g_sys_state == SYS_STATE_TASK_RUNNING)
        {
            switch (g_selected_task) 
            {
                case 1: Task1_Run(); break;
                case 2: Task2_Run(); break;
                case 3: Task3_Run(); break;
                case 4: Task4_Run(); break;
            }
        }
    }
}
"""
    new_lines = lines[:start_idx] + [new_logic + "\n    System_Scheduler();\n"] + lines[end_idx:]
    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(new_lines)
    
    with open(r"d:\KeilMDKARM5.35\smart-car\App\task1.c", "w", encoding="utf-8") as f:
        f.writelines([
            '#include "task.h"\n',
            '#include "main.h"\n',
            '#include "encoder.h"\n',
            '#include "motor.h"\n',
            '#include "mpu6050.h"\n',
            '#include "openmv.h"\n',
            '#include "infrared.h"\n',
            '#include "syn6658.h"\n',
            '#include "alert.h"\n',
            '\nextern StateMachine_TypeDef g_state_machine;\n',
            'extern MPU6050_DataTypeDef g_mpu_data;\n',
            'extern IR_DataTypeDef g_ir_data;\n',
            'extern volatile int16_t g_encoder_left;\n',
            'extern volatile int16_t g_encoder_right;\n',
            'extern uint32_t HAL_GetTick(void);\n',
            'extern void delay_ms(uint32_t);\n',
            'extern void LineFollow_CalibrateGyroBias(void);\n',
            'extern CarEvent_TypeDef DetectEvent(void);\n',
            'extern void LineFollow_RunByRawIR(void);\n',
            'extern void Debug_LogIR(CarEvent_TypeDef event);\n\n',
            'void Task2_Run(void) { while(1) delay_ms(10); }\n',
            'void Task3_Run(void) { while(1) delay_ms(10); }\n',
            'void Task4_Run(void) { while(1) delay_ms(10); }\n\n'
        ])
        
        # 任务一：结合路段状态机
        f.write('''
/* ====== 任务1 状态机：A->B->C->D->A ====== */
typedef enum {
    M_STAGE_A_TO_B = 0,
    M_STAGE_B_TURN,
    M_STAGE_B_TO_C,
    M_STAGE_C_TURN,
    M_STAGE_C_TO_D,
    M_STAGE_D_TURN,
    M_STAGE_D_TO_A,
    M_STAGE_FINISH
} MissionStage_t;

void Blink_LED_PC13(void)
{
    // 初始化PC13为输出
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 亮灯
    delay_ms(200);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);   // 灭灯
}

void Task1_Run(void) 
{
    // 初始化原版所需
    SM_Init(&g_state_machine, 3); 
    MPU6050_ResetYaw(&g_mpu_data);
    LineFollow_CalibrateGyroBias();
    IR_ResetTracking();
    SM_Process(&g_state_machine, EVENT_START); 
    
    // 初始化外层里程逻辑
    MissionStage_t m_stage = M_STAGE_A_TO_B;
    int32_t total_distance = 0; // 用编码器累加表示距离
    
    // 起点A播报
    SYN6658_Speak("\\xC6\\xF0\\xB5\\xE3\\x41\\xB5\\xE3"); // "起点A点"
    Blink_LED_PC13();
    
    MPU6050_DataTypeDef last_mpu;
    MPU6050_ReadAll(&last_mpu);

    // 主循环
    while(1)
    {
        IR_Read(&g_ir_data);
        MPU6050_ReadAll(&g_mpu_data);
        
        CarEvent_TypeDef event = DetectEvent(); 
        
        // 累加左轮作为参考里程 (需根据您的N20编码器线数修正这个阈值，这里假设大约100对应100cm，请实际测试更改)
        total_distance += g_encoder_left; 
        
        /* 外层任务1逻辑监控 */
        if (m_stage == M_STAGE_A_TO_B && total_distance > 5000) // 假设阈值5000对应到达B
        {
            SYN6658_Speak("\\xB5\\xBD\\xB4\\xEF\\x42\\xB5\\xE3"); // 到达B点
            Blink_LED_PC13();
            m_stage = M_STAGE_B_TURN;
            total_distance = 0;
            // TODO: 如果需要，可以切换偏航角目标或者控制电机进行弯道掉头
        }
        else if (m_stage == M_STAGE_B_TURN) // 假设您根据MPU或者外部信号判断转角完毕
        {
            // if 转弯结束...
            m_stage = M_STAGE_B_TO_C;
            total_distance = 0;
        }
        else if (m_stage == M_STAGE_B_TO_C && total_distance > 5000) 
        {
            SYN6658_Speak("\\xB5\\xBD\\xB4\\xEF\\x43\\xB5\\xE3"); // 到达C点
            Blink_LED_PC13();
            m_stage = M_STAGE_C_TURN;
            total_distance = 0;
        }
        else if (m_stage == M_STAGE_C_TURN) 
        {
            m_stage = M_STAGE_C_TO_D;
            total_distance = 0;
        }
        else if (m_stage == M_STAGE_C_TO_D && total_distance > 5000) 
        {
            SYN6658_Speak("\\xB5\\xBD\\xB4\\xEF\\x44\\xB5\\xE3"); // 到达D点
            Blink_LED_PC13();
            m_stage = M_STAGE_D_TURN;
            total_distance = 0;
        }
        else if (m_stage == M_STAGE_D_TURN) 
        {
            m_stage = M_STAGE_D_TO_A;
            total_distance = 0;
        }
        else if (m_stage == M_STAGE_D_TO_A && total_distance > 5000) 
        {
            SYN6658_Speak("\\xBB\\xD8\\xB5\\xBD\\x41\\xB5\\xE3"); // 回到A点停车
            Blink_LED_PC13();
            Motor_SetSpeed(0, 0); 
            m_stage = M_STAGE_FINISH;
        }

        if (m_stage != M_STAGE_FINISH) 
        {
            SM_Process(&g_state_machine, event);
            Debug_LogIR(event);
            if (g_state_machine.current_state == STATE_LINE_FOLLOW)
            {
                LineFollow_RunByRawIR();
            }
        }
        
        delay_ms(10); 
    }
}
''')

    print("Patched main.c and Created task1.c")

rewrite()
