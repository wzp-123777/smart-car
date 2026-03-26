import os

file_path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
    text = f.read()

import re

# find uint32_t hold_time
text = re.sub(r'uint32_t hold_time;', r'uint32_t hold_time=0;', text)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(text)
