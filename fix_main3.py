import re
file_path = r'd:\KeilMDKARM5.35\smart-car\App\main.c'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()
# let's just show line 695 to 800

for i, l in enumerate(lines[695:750]):
    print(f"{i+696}: {l}", end='')

