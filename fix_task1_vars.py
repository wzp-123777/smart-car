with open(r'd:\\KeilMDKARM5.35\\smart-car\\App\\task1.c', 'r', encoding='utf-8') as f:
    text = f.read()

adds = '''extern void LineFollow_RunByRawIR(void);
extern IR_DataTypeDef g_ir_data;
extern MPU6050_DataTypeDef g_mpu_data;
extern StateMachine_TypeDef g_state_machine;
'''

text = text.replace('extern void LineFollow_RunByRawIR(void);', adds)
text = text.replace('evt = 99;', 'evt = EVENT_NONE; // fallback')

with open(r'd:\\KeilMDKARM5.35\\smart-car\\App\\task1.c', 'w', encoding='utf-8') as f:
    f.write(text)

print('Added task1.c externs')
