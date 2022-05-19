from cv2 import CV_32F
import numpy as np
import cv2
import imutils

video_path = ''
# cap = cv2.VideoCapture(0)

img_path = 'imgs_to_test/scene_00.jpeg'
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

# Show video or image
while True:
    ##Capture frame by frame    
    ##Uncomment the following line to read the video
    #ret, frame = cap.read()
        
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #color range selection
    lower_blueT = cv2.getTrackbarPos("Lower Blue", "Trackbars")
    upper_blueT = cv2.getTrackbarPos("Upper Blue", "Trackbars")
    
    lower_greenT = cv2.getTrackbarPos("Lower Green", "Trackbars")
    upper_greenT = cv2.getTrackbarPos("Upper Green", "Trackbars")
    
    lower_redT = cv2.getTrackbarPos("Lower Red", "Trackbars")
    upper_redT = cv2.getTrackbarPos("Upper Red", "Trackbars")

 
    # Setting ideal values for ranges in fire detection
    lower= np.array([0, 0, 81])
    upper = np.array([97, 90, 255])

    mask = cv2.inRange(img, (lower), (upper))
    result = cv2.bitwise_and(img, img, mask=mask)

    #mask = cv2.inRange(img, (lower_blueT, lower_greenT, lower_redT), (upper_blueT, upper_greenT, upper_redT)) # Com trackbar
    cv2.imshow("mask", mask)
    cv2.imshow('Image', img)
    cv2.imshow('Image2', result)

    #Creating the mask for color detection
    
    if cv2.waitKey(1) == ord('p'):
        cv2.destroyAllWindows()


    # Finding the centroid


    # Achando os contornos
    #hsv = cv2.cvtColor(img, cv2.Color_BGR2HSV)
    cnt = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnt = imutils.grab_contours(cnt)
    #Calculando os momentos
    for i in cnt: 
        area = cv2.contourArea(i)
        if area> 500:
            cv2.drawContours(img, [cnt], -1, (0,255,0), 3)

        M = cv2.moments(i)
        #Centroid
        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])
        #Criando um ponto para representar o centroid
        cv2.circle(img, (cx,cy), 7, (255, 255, 255), -1)
        cv2.putText(img, "Centro", (cx-20, cy-20), cv2FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        cv2.imshow("Centroid", img)
    
    print("centroid está em ", cx, cy)

    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break


cap.release()
cv2.destroyAllWindows()
