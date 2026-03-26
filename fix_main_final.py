import os
import re

file_path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
    text = f.read()

get_key_event = '''
KeyEvent_t Get_Key_Event(void) 
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
}
'''

text = re.sub(r'KeyEvent_t Get_Key_Event\(void\).*?return KEY_EVENT_NONE;\s*\}', get_key_event.strip(), text, flags=re.DOTALL)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(text)
