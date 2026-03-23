import sys
from PIL import Image

for file_path in sys.argv[1:]:
    try:
        # 打开错误的图片格式并转换为 8位灰度 (L) 
        img = Image.open(file_path).convert('L')
        # 强制将后缀替换为 .pgm 并保存
        new_path = file_path.rsplit('.', 1)[0] + '.pgm'
        img.save(new_path)
        print(f"转换成功: {file_path} -> {new_path}")
    except Exception as e:
        print(f"转换失败 {file_path}: {e}")
