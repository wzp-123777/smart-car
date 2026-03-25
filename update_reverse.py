import re

def update_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Left turn
    content = re.sub(
        r'/\* error < 0 代表黑线偏左（车体偏右），需左转，.*?\*/\s*inner_speed = 300 - \(int16_t\)\(-total_adjust\);\s*if \(inner_speed < 0\) \{ inner_speed = 0; \}',
        '/* error < 0 代表黑线偏左（车体偏右），需左转，内侧轮可反转以减小转弯半径 */\n        inner_speed = 300 - (int16_t)(-total_adjust);\n        if (inner_speed < -300) { inner_speed = -300; } /* 允许反向 */',
        content
    )

    # Right turn
    content = re.sub(
        r'/\* error > 0 代表黑线偏右（车体偏左），需右转，.*?\*/\s*inner_speed = 300 - \(int16_t\)\(total_adjust\);\s*if \(inner_speed < 0\) \{ inner_speed = 0; \}',
        '/* error > 0 代表黑线偏右（车体偏左），需右转，内侧轮可反转以减小转弯半径 */\n        inner_speed = 300 - (int16_t)(total_adjust);\n        if (inner_speed < -300) { inner_speed = -300; } /* 允许反向 */',
        content
    )

    # Update summary comment
    content = content.replace(
        '控制手段: 仅对内侧轮进行减速以形成差速转向（内侧快卡死时差速最大）。',
        '控制手段: 仅对内侧轮进行减速甚至反转以形成更大的差速转向。'
    )

    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)

update_file(r'd:\KeilMDKARM5.35\smart-car\App\main.c')
update_file(r'd:\KeilMDKARM5.35\smart-car\MY\main.c')
print('Updated main.c for reverse-phase turning')
