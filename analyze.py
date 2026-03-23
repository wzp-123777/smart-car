import os
import sys
from PIL import Image
import numpy as np

folder = r"C:\Users\wuzuping\Desktop\smartcar图像"
files_to_check = ["打火机", "剪刀", "锤子.png"]

print("==== 开始分析真实赛场图形特征 ====")
for f in files_to_check:
    path = os.path.join(folder, f)
    if not os.path.exists(path):
        print(f"找不到: {path}")
        continue
        
    try:
        img = Image.open(path).convert('L')
        arr = np.array(img)
        
        # 色块是以深色为主（白色背景），如果颜色浅的（大于200），认为是背景。
        # 寻找非背景像素 (<200)
        object_pixels = np.argwhere(arr < 200)
        
        if len(object_pixels) == 0:
            print(f"[{f}]: 全是白的？")
            continue
            
        y_min, x_min = object_pixels.min(axis=0)
        y_max, x_max = object_pixels.max(axis=0)
        
        # 宽和高
        w = x_max - x_min + 1
        h = y_max - y_min + 1
        area = w * h
        actual_pixels = len(object_pixels)
        
        density = actual_pixels / area
        ratio = w / h
        
        print(f"[{f}] 分析完毕:")
        print(f"  --> 长宽比(宽/高): {ratio:.2f} (宽{w}, 高{h})")
        print(f"  --> 密度指数:       {density:.2f}")
        
    except Exception as e:
        print(f"报错 {f}: {e}")
