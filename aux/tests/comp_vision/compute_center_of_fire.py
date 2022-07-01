import numpy as np
import cv2

## Compute the 'center of gravity' of the fire in the image
'''
    Steps
    -> Input a binary image
    1. Compute the contours using binary image
    2. Use cv2.moments to compute the moments of contours
    3. Compute centroids
'''

img_path = 'imgs_to_test/bin_img_00.jpeg'
img = cv2.imread(img_path)
bin_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

contours, hierarchy = cv2.findContours( bin_img,
                                        cv2.RETR_TREE,
                                        cv2.CHAIN_APPROX_NONE)


#draw contours to visualize the results
cv2.drawContours(   image = img,
                    contours = contours,
                    contourIdx = 0,
                    color = (255,0,0), thickness = 3)

cv2.imshow('Image', img)
cv2.waitKey(0)            