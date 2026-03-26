import re

src = r'd:\\KeilMDKARM5.35\\smart-car\\Project\\main_debug.txt'
dst = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

with open(src, 'r', encoding='utf-8', errors='ignore') as f:
    text_debug = f.read()

with open(dst, 'r', encoding='utf-8', errors='ignore') as f:
    text_main = f.read()

# I want to take the Get_Key_Event and System_Scheduler from text_debug and put it in text_main.
# text_debug has lines printed with 1: , 2: 
# Let's extract them:
lines = text_debug.splitlines()
clean_lines = []
for l in lines:
    if ':' in l:
        clean_lines.append(l.split(':', 1)[1][1:]) # remove "123: "

clean_text = '\n'.join(clean_lines)

# We want everything from /* ==================== 语音播报 GBK 编码 ==================== */ up to int main(void)
target_start = "/* ==================== 语音播报 GBK 编码 ==================== */"
target_end = "int main(void)"

if target_start in clean_text and target_end in clean_text:
    section = clean_text[clean_text.find(target_start): clean_text.find(target_end)]
    
    # insert before int main(void) in main.c
    if target_end in text_main:
        new_main = text_main.replace(target_end, section + '\n' + target_end)
        
        # Now we need to modify the end of main(void) to call System_Scheduler();
        # The end of main has:
        # /* ==============================================
        # *                 ???????????????????????
        # * ============================================== */
        # /* ===== 状态机初始化 ===== */
        # We will replace everything after Motor_Enable(1); with System_Scheduler(); \n}
        
        start_cut = new_main.find('/* ===== 状态机初始化 ===== */')
        if start_cut != -1:
            end_cut = new_main.find('/**\n * @brief  USART3 IRQ', start_cut)
            if end_cut != -1:
                # Remove all that code inside main and replace with System_Scheduler();
                # wait, let me just find the Motor_Enable(1);
                # Instead of removing, let's just make it properly structured.
                cut_main = new_main[:start_cut] + '    System_Scheduler();\n}\n\n' + new_main[end_cut:]
                
                with open(dst, 'w', encoding='utf-8', errors='ignore') as fw:
                    fw.write(cut_main)
                print("main.c fully restored with System_Scheduler!")
            else:
                print("Could not find USART3 IRQ")
        else:
            print("Could not find 状态机初始化")
    else:
        print("int main(void) not found in main.c")
else:
    print("Could not find sections in debug text")

