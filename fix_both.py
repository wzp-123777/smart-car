with open(r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c', 'r', encoding='utf-8') as f:
    text = f.read()

text = text.replace('static void LineFollow_RunByRawIR(void)', 'void LineFollow_RunByRawIR(void)')

with open(r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c', 'w', encoding='utf-8') as f:
    f.write(text)

with open(r'd:\\KeilMDKARM5.35\\smart-car\\App\\task1.c', 'r', encoding='utf-8') as f:
    text2 = f.read()

text2 = text2.replace('EVENT_CROSSROAD', '99')
text2 = text2.replace('delay_ms', 'Delay_ms')

header_adds = '''#include "stm32f4xx.h"
extern void LineFollow_RunByRawIR(void);
'''
text2 = text2.replace('#include "stm32f4xx.h"', header_adds)

with open(r'd:\\KeilMDKARM5.35\\smart-car\\App\\task1.c', 'w', encoding='utf-8') as f:
    f.write(text2)

print('Fixed both files!')
