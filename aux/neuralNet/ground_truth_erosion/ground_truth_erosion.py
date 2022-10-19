import cv2
import numpy as np
from os import listdir
from os.path import isfile

def erode(img):

    kernel = np.ones((5,5), np.uint8)

    img_erosion = cv2.erode(src = img, kernel = kernel, iterations = 1)
    return img_erosion


for file in listdir('ground_truth'):
    
    
    img = cv2.imread(f'ground_truth/{file}')
    img_erosion = erode(img)

    
    cv2.imwrite(filename = file, img = img_erosion)
    print(f'File {file} saved!')
    
    


