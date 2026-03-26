import re
file_path = r'd:\KeilMDKARM5.35\smart-car\App\main.c'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Fix C89 issue in main.c line 711
content = content.replace('''    else 
    {
        if (key_pressed == 1) 
        {
            uint32_t hold_time;''', '''    else 
    {
        if (key_pressed == 1) 
        {
            uint32_t hold_time;
''')

# Well, let me just replace the whole Get_Key_Event
old_get_key_event = '''KeyEvent_t Get_Key_Event(void) 
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
}'''

new_get_key_event = '''KeyEvent_t Get_Key_Event(void) 
{
    static uint32_t key_down_tick = 0;
    static uint8_t  key_pressed = 0;
    uint32_t hold_time = 0;
    
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
}'''
content = content.replace(old_get_key_event, new_get_key_event)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)
print("main.c Get_key_event fixed")
