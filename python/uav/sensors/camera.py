import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#TODO
'''
    1. Adicionar as funções de detecção de cores
'''
class uavCamera:
    
    #Colors of interest color = [blue, green, red]
    # or color_hsv = [h, s, v]
    
    red_trees_bgr =  [0, 0, 0]
    yellow_trees_bgr = [0, 0, 0]
    green_trees_bgr = [0, 0, 0]

    
    def __init__(   self,
                    img_width: int,
                    img_height: int,
                    node_name: str,
                    subscriber_name: str):
        

        self.img_width, self.img_height = img_width, img_height
        self.node_name = node_name
        self.subscriber_name = subscriber_name
        self.bridge = CvBridge()        
        self.sub = rospy.Subscriber(name = self.subscriber_name,
                                        data_class = Image,
                                        callback = self.img_msg_to_cv)


        self.cv_img = np.zeros((img_height, img_width, 3), dtype= np.uint8)
        self.blurred_img = cv2.GaussianBlur( self.cv_img, (5,5), 0)
        self.cv_img_hsv = cv2.cvtColor(self.blurred_img, cv2.COLOR_BGR2HSV)
        
        self.i_see_fire: bool = False

        

    # img(ROS) -> img(OpenCV)
    def img_msg_to_cv(self, img_msg: Image) -> None:
        
        # msg.Image -> 'cv2.Image'
        cv_img_msg = self.bridge.imgmsg_to_cv2(img_msg, 'passthrough')
        #Color correction
        self.cv_img = cv2.cvtColor(cv_img_msg, cv2.COLOR_BGR2RGB)

    # Subscribe and image frame update
    def update_img_cv(self) -> None:
        
        '''
            img_msg: image input from ROS
        '''
        #  to camera in Gazebo and apply img_msg_to_cv() as callback function
        # self.img_msg = rospy.Subscriber(name = self.subscriber_name,
        #                                 data_class = Image,
        #                                 callback = self.img_msg_to_cv)

        #Resize image
        self.cv_img = cv2.resize(self.cv_img, (self.img_height, self.img_width))
        
        # self.cv_img_hsv = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
        
    def color_detection(    self,
                            img_src: np.array) -> None:
        
        lower_values_red = np.array([0, 103, 45])
        upper_values_red = np.array([12, 255, 255])
        
        lower_values_yellow= np.array([26, 130, 88])
        upper_values_yellow = np.array([39, 193, 172])
        
        _img = np.copy(img_src)
        
        _blurred_img = cv2.GaussianBlur( _img, (5,5), 0)

        _cv_img_hsv = cv2.cvtColor(_blurred_img, cv2.COLOR_BGR2HSV)

        self.mask_red = cv2.inRange( _cv_img_hsv, lower_values_red, upper_values_red)
        self.cv_img_masked_red = cv2.bitwise_and( _img, _img, mask= self.mask_red)
        
        self.mask_yellow = cv2.inRange( _cv_img_hsv, lower_values_yellow, upper_values_yellow)
        self.cv_img_masked_yellow = cv2.bitwise_and( _img, _img, mask= self.mask_yellow)

                
    # Centroid calculations for the fire (red)
    def find_centroid(self) -> None:
                            
        countours, hierarchy = cv2.findContours(self.mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours( self.mask_red, countours, 0, (0,255,0), 3)
        
        for countour in countours: 
            self.red_area = cv2.contourArea(countour)
            #print(self.area)
            if self.red_area>10000: 
                self.red_moment = cv2.moments(countour)
                # position of the centroid
                self.red_cx = int(self.red_moment["m10"]/self.red_moment["m00"]) 
                self.red_cy = int(self.red_moment["m01"]/self.red_moment["m00"])
                #  Creating a point to represent the centroid 
                cv2.circle( self.cv_img, (self.red_cx, self.red_cy), 7, (255, 255, 255), -1)

    




    def display_img(self):
        cv2.imshow("uav View", self.cv_img)
        cv2.imshow("uav Mask", self.cv_img_masked_red)
        cv2.imshow("uav Mask", self.cv_img_masked_yellow)
        
        

    def update_state(self) -> None:        
        self.sub = rospy.Subscriber(name = self.subscriber_name,
                                        data_class = Image,
                                        callback = self.img_msg_to_cv)
        
        self.color_detection(img_src = self.cv_img)
        self.find_centroid()
        
