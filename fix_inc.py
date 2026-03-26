path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\task1.c'
with open(path, 'r', encoding='utf-8') as f:
    text = f.read()

text = text.replace('#include "main.h"', '#include "stm32f4xx.h"\n#include "delay.h"')

with open(path, 'w', encoding='utf-8') as f:
    f.write(text)
print('Fixed task1.c includes')
