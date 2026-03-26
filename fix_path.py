import os
file_path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\task1.c'
with open(file_path, 'r', encoding='utf-8') as f:
    text = f.read()

text = text.replace('#include "main.h"', '#include "../MY/main.h"')

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(text)
