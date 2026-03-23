import os

print('--- E Drive Contents ---')
for root, dirs, files in os.walk('E:\\'):
    for name in files:
        path = os.path.join(root, name)
        size = os.path.getsize(path)
        print(f"{path} ({size} bytes)")
