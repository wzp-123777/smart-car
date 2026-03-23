import image, os
try:
    print('Testing images...')
    # Check what keypoints we can extract
    i1 = image.Image('/1.pgm')
    k1 = i1.find_keypoints()
    print('1.pgm keypoints:', len(k1) if k1 else 0)
    
    i_h = image.Image('/hammer.pgm')
    k_h = i_h.find_keypoints()
    print('hammer.pgm keypoints:', len(k_h) if k_h else 0)

except Exception as e:
    print('Error:', e)
