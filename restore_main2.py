import shutil

src = r'd:\\KeilMDKARM5.35\\smart-car\\Project\\main_debug.txt'
dst = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

shutil.copyfile(src, dst)
