import numpy as np
import cv2

##TODO:
'''
    1. Criar uma função 'gradiente de vermelho'/densidade de vermelho 
'''

video_path = ''
# cap = cv2.VideoCapture(0)

img_path = './imgs_to_test/scene_00.jpeg'
img = cv2.imread(img_path)

def nothing(x):
    pass

##Trackbars
cv2.namedWindow("Trackbars")
cv2.createTrackbar("Lower Blue", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Upper Blue", "Trackbars", 0, 255, nothing)

cv2.createTrackbar("Lower Green", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Upper Green", "Trackbars", 0, 255, nothing)

cv2.createTrackbar("Lower Red", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Upper Red", "Trackbars", 0, 255, nothing)

#dilate and erode
cv2.createTrackbar("Erosion Kernel", "Trackbars", 0, 10, nothing)
cv2.createTrackbar("Dilation Kernel", "Trackbars", 0, 10, nothing)

# Show video or image
while True:
    ##Capture frame by frame    
    ##Uncomment the following line to read the video
    #ret, frame = cap.read()
    
    #Clean origi image before applying contours and other transformations
    img = cv2.imread(img_path)

    #color range selection
    lower_blue = cv2.getTrackbarPos("Lower Blue", "Trackbars")
    upper_blue = cv2.getTrackbarPos("Upper Blue", "Trackbars")
    
    lower_green = cv2.getTrackbarPos("Lower Green", "Trackbars")
    upper_green = cv2.getTrackbarPos("Upper Green", "Trackbars")
    
    lower_red = cv2.getTrackbarPos("Lower Red", "Trackbars")
    upper_red = cv2.getTrackbarPos("Upper Red", "Trackbars")

    erosion_kernel_size  = cv2.getTrackbarPos("Erosion Kernel", "Trackbars")
    dilate_kernel_size  = cv2.getTrackbarPos("Dilation Kernel", "Trackbars")
        
    
    mask = cv2.inRange(img, (lower_blue, lower_green, lower_red), (upper_blue, upper_green, upper_red))
    mask_blur = cv2.GaussianBlur(src = mask, ksize= (3,3), sigmaX = 0)

    erosion_kernel = np.ones((erosion_kernel_size, erosion_kernel_size), np.uint8)
    mask_erosion = cv2.erode(mask_blur, kernel = erosion_kernel, iterations=1)

    dilate_kernel = np.ones((dilate_kernel_size, dilate_kernel_size), np.uint8)
    mask_dilate = cv2.dilate(src = mask_erosion, kernel = dilate_kernel, iterations = 1)

    #erosion and dilation
    # element = cv2.getStructuringElement(shape, ksize)
    # erosion_dst = cv2.erode(src = mask, kernel = erosion_kernel)
    # dilation_dst = cv2.erode(src = erosion_dst, kernel = dilation_kernel)

    #contours
    contours, _ = cv2.findContours( image = mask_dilate,
                                    mode = cv2.RETR_TREE,
                                    method = cv2.CHAIN_APPROX_NONE)
    
    
    cv2.drawContours(   image = img,
                        contours = contours,
                        contourIdx = -1,
                        color = (255,0,0),
                        thickness = 3)
                                        
    
    cv2.imshow("mask", mask_dilate)
    cv2.imshow('Image', img)
    
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break


# cap.release()
cv2.destroyAllWindows()