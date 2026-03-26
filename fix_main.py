import re
file_path = r'd:\KeilMDKARM5.35\smart-car\App\main.c'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# I see the problem. The generated code /* ==================== 语音播报 GBK 编码 ==================== */ and typedef enums were put INSIDE main(void)!!
# Let's extract everything from /* ==================== 语音播报 GBK 编码 ==================== */ to oid System_Scheduler(void) and put it ABOVE main(void).
# Wait, System_Scheduler is also inside main? No, it's defined after main... no, it's inside main because main has NO closing brace before System_Scheduler! 
# Let me just properly place the closing brace of main(void).
# main(void) calls System_Scheduler() at the end, and then has }.
# Wait, if main(void) calls System_Scheduler() then System_Scheduler(); and } are the end of main(void).
# The functions Get_Key_Event and System_Scheduler must be moved ABOVE main(void).

match = re.search(r'(int main\(void\).*?/\* ==============================================\n\*                 \?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\?\n\* ============================================== \*/)', content, flags=re.DOTALL)
if match:
    main_prologue = match.group(1)
    
    # We will just rewrite the end of the file.
    
    content_before_main = content[:match.start()]
    
    new_tail = '''
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
            uint32_t hold_time;
            key_pressed = 0; 
            hold_time = HAL_GetTick() - key_down_tick;
            
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
'''
    content_to_write = content_before_main + new_tail + '\n' + main_prologue + '\n    System_Scheduler();\n}\n'
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content_to_write)
    print("Fixed!")
else:
    print("Could not match main prologue.")
