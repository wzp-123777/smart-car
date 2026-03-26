import os

file_path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
    text = f.read()

import re
print('Has System_Scheduler:', 'System_Scheduler' in text)
print('Has main block end?', text.find('int main(void)'))

# The int main(void) is on line 628. Let's see what is on line 711
