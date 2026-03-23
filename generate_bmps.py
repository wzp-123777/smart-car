import os
from PIL import Image

src_dir = r'C:\Users\wuzuping\Desktop\smartcar图像'
dest_dir = r'E:\\'

# ORB特征点提取需要适当的细节，120x120足够了
TARGET_SIZE = (120, 120)

files_to_convert = {
    '打火机': 'lighter.bmp',
    '剪刀': 'scissors.bmp',
    '锤子.png': 'hammer.bmp'
}

for src_name, dest_name in files_to_convert.items():
    src_path = os.path.join(src_dir, src_name)
    dest_path = os.path.join(dest_dir, dest_name)

    if os.path.exists(src_path):
        try:
            # 打开原图，转化为RGB避免PNG透明通道报错，再转灰度图
            img = Image.open(src_path).convert('RGBA').convert('L')
            
            # 使用thumbnail保持比例缩小
            img.thumbnail(TARGET_SIZE, Image.Resampling.LANCZOS)
            
            # ORB不强制要求正方形，直接保存裁掉多余白边的图即可
            img.save(dest_path, format='BMP')
            print(f'Successfully generated ORB target: {src_name} -> {dest_path}')
        except Exception as e:
            print(f'Error processing {src_name}: {e}')
    else:
        print(f'File missing: {src_path}')
