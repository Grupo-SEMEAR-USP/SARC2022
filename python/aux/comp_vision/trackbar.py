import numpy as np
import cv2


video_path = ''
# cap = cv2.VideoCapture(0)

img_path = './imgs_to_test/scene_00.jpeg'
img = cv2.imread(img_path)

def nothing(x):
    pass

#Trackbars
cv2.namedWindow("Trackbars")
cv2.createTrackbar("Lower Blue", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Upper Blue", "Trackbars", 0, 255, nothing)

cv2.createTrackbar("Lower Green", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Upper Green", "Trackbars", 0, 255, nothing)

cv2.createTrackbar("Lower Red", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("Upper Red", "Trackbars", 0, 255, nothing)

# Show video or image
while True:
    ##Capture frame by frame    
    ##Uncomment the following line to read the video
    #ret, frame = cap.read()
        
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #color range selection
    lower_blue = cv2.getTrackbarPos("Lower Blue", "Trackbars")
    upper_blue = cv2.getTrackbarPos("Upper Blue", "Trackbars")
    
    lower_green = cv2.getTrackbarPos("Lower Green", "Trackbars")
    upper_green = cv2.getTrackbarPos("Upper Green", "Trackbars")
    
    lower_red = cv2.getTrackbarPos("Lower Red", "Trackbars")
    upper_red = cv2.getTrackbarPos("Upper Red", "Trackbars")

    # lower_gray = cv2.getTrackbarPos("Lower Value", "Trackbars")
    # upper_gray = cv2.getTrackbarPos("Upper Value", "Trackbars")
    
    
    mask = cv2.inRange(img, (lower_blue, lower_green, lower_red), (upper_blue, upper_green, upper_red))

    cv2.imshow("mask", mask)
    cv2.imshow('Image', img)
    
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break


cap.release()
cv2.destroyAllWindows()