import os

file_path = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
    text = f.read()

import re

# System_Scheduler in main.c is gone?!
# git checkout should have restored it! Let's check status!
