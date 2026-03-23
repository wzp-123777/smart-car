import sys
from PIL import Image

try:
    path = "E:\\4.bmp"
    print("正在处理打火机第二模板...")
    img = Image.open(path).convert('L')
    
    # 比赛标准小模板，将其粗略缩放到合适的识别比例以匹配远处的图案
    # 为了避免把它压太扁，按照长宽比例来微调一下
    w, h = img.size
    img = img.resize((int(w * 0.5), int(h * 0.5)), Image.Resampling.LANCZOS)
    
    img.save("E:\\4.pgm")
    print("生成第二打火机特征图 E:\\4.pgm 成功！(已自动灰度化+降准)")
except Exception as e:
    print(f"处理 4.bmp 发生错误: {e}，请检查这文件是不是已经被打开占用了")
