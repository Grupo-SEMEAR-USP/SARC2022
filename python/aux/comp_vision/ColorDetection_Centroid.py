from cv2 import CV_32F
import numpy as np
import cv2
import imutils

video_path = ''
# cap = cv2.VideoCapture(0)

img_path = 'imgs_to_test/scene_01.jpeg'
img = cv2.imread(img_path)


def nothing(x):
    pass

##Trackbars RGB
#cv2.namedWindow("Trackbars")
#cv2.createTrackbar("Lower Blue", "Trackbars", 0, 255, nothing)
#cv2.createTrackbar("Upper Blue", "Trackbars", 0, 255, nothing)

#cv2.createTrackbar("Lower Green", "Trackbars", 0, 255, nothing)
#cv2.createTrackbar("Upper Green", "Trackbars", 0, 255, nothing)

#cv2.createTrackbar("Lower Red", "Trackbars", 0, 255, nothing)
#cv2.createTrackbar("Upper Red", "Trackbars", 0, 255, nothing)

## Trackbars HSV
global H_low,H_high,S_low,S_high,V_low,V_high

#create a seperate window named 'controls' for trackbar
cv2.namedWindow('controls',2)
cv2.resizeWindow("controls", 550,10);


#global variable
H_low = 0
H_high = 179
S_low= 0
S_high = 255
V_low= 0
V_high = 255

#create trackbars for high,low H,S,V 
cv2.createTrackbar('low H','controls',0,179,nothing)
cv2.createTrackbar('high H','controls',179,179,nothing)

cv2.createTrackbar('low S','controls',0,255,nothing)
cv2.createTrackbar('high S','controls',255,255,nothing)

cv2.createTrackbar('low V','controls',0,255,nothing)
cv2.createTrackbar('high V','controls',255,255,nothing)


# Show video or image
while True:
    ##Capture frame by frame    
    ##Uncomment the following line to read the video
    #ret, frame = cap.read()
    
    #Filtragem da imagem
    blurred_img = cv2.GaussianBlur(img, (5,5), 0)

    hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #color range selection RGB
    #lower_blueT = cv2.getTrackbarPos("Lower Blue", "Trackbars")
    #upper_blueT = cv2.getTrackbarPos("Upper Blue", "Trackbars")
    
    #lower_greenT = cv2.getTrackbarPos("Lower Green", "Trackbars")
    #upper_greenT = cv2.getTrackbarPos("Upper Green", "Trackbars")
    
    #lower_redT = cv2.getTrackbarPos("Lower Red", "Trackbars")
    #upper_redT = cv2.getTrackbarPos("Upper Red", "Trackbars")

    #color range selection HSV    
    lower_H = cv2.getTrackbarPos("low H", "controls")
    upper_H = cv2.getTrackbarPos("high H", "controls")
    
    lower_S = cv2.getTrackbarPos("low S", "controls")
    upper_S = cv2.getTrackbarPos("high S", "controls")

    lower_V = cv2.getTrackbarPos("low V", "controls")
    upper_V = cv2.getTrackbarPos("high V", "controls")

    # Setting ideal values for ranges in fire detection RGB
    lower= np.array([0, 0, 81])
    upper = np.array([97, 90, 255])

    #mask = cv2.inRange(img, (lower_blueT, lower_greenT, lower_redT), (upper_blueT, upper_greenT, upper_redT)) # Com trackbar
    mask = cv2.inRange(img, (lower), (upper))
    result = cv2.bitwise_and(img, img, mask=mask)
    
    # cv2.imshow("mask", mask)
    # cv2.imshow('Image', img)
    # cv2.imshow('Image2', result)

    # Setting ideal values for ranges in fire detection HSV
    lower_hsv= np.array([0, 103, 45])
    upper_hsv = np.array([12, 255, 255])

    #mask_hsv = cv2.inRange(hsv, (lower_H, lower_S, lower_V), (upper_H, upper_S, upper_V))
    mask_hsv = cv2.inRange(hsv, (lower_hsv), (upper_hsv))
    result_hsv = cv2.bitwise_and(img, img, mask=mask_hsv)

    cv2.imshow("mask", mask_hsv)
    cv2.imshow('Image', img)
    cv2.imshow('Image2', result_hsv)

   

    # #Creating the mask for color detection
    
    # if cv2.waitKey(1) == ord('p'):
    #     cv2.destroyAllWindows()


    # Finding the centroid


    ##Achando os contornos

    # Primeira forma

    countours, hierarchy= cv2.findContours(mask_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, countours, -1, (0,255,0), 3)
    for countour in countours: 
        area = cv2.contourArea(countour)
        print(area)
        if area>7000: 
            M = cv2.moments(countour)
            #Centroid
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
            #Criando um ponto para representar o centroid
            cv2.circle(img, (cx,cy), 7, (255, 255, 255), -1)
            #cv2.putText(img, "Centro", (cx-20, cy-20), cv2FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            cv2.imshow("Centroid", img)




    # Segunda forma

    #cnt = cv2.findContours(mask_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cnt = imutils.grab_contours(cnt)
    #Calculando os momentos
    # for i in cnt: 
    #     area = cv2.contourArea(i)
    #     if area>0.5:
    #         print (area)
    #         cv2.drawContours(img, [i], -1, (0,255,0), 3)

    #     M = cv2.moments(i)
    #     #Centroid
    #     cx = int(M["m10"]/M["m00"])
    #     cy = int(M["m01"]/M["m00"])
    #     #Criando um ponto para representar o centroid
    #     cv2.circle(img, (cx,cy), 7, (255, 255, 255), -1)
    #     cv2.putText(img, "Centro", (cx-20, cy-20), cv2FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    #     cv2.imshow("Centroid", img)
    
    # print("centroid está em ", cx, cy)

    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break


cap.release()
cv2.destroyAllWindows()
