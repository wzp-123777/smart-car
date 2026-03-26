import shutil

src = r'd:\\KeilMDKARM5.35\\smart-car\\main_debug.txt'
dst = r'd:\\KeilMDKARM5.35\\smart-car\\App\\main.c'

shutil.copyfile(src, dst)
print("main.c restored from backup!")
