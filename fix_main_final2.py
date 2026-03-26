import re

file_path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
    text = f.read()

# find uint32_t start_run_time = HAL_GetTick();
text = text.replace('uint32_t start_run_time = HAL_GetTick();', '{uint32_t start_run_time = HAL_GetTick();')
text = text.replace('System_Scheduler();\r\n}', 'System_Scheduler();\r\n}}')

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(text)
