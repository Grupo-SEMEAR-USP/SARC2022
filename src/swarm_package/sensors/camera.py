#!/usr/bin/env python3

# Importing Python Files
from swarm_package import helper

# Import Libraries
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
import cv2

# Importing Ros Messages
from sensor_msgs.msg import Image


class uavCamera:
       
    red_trees_bgr =  [0, 0, 0]
    yellow_trees_bgr = [0, 0, 0]
    green_trees_bgr = [0, 0, 0]

    def __init__(   self,
                    img_width: int,
                    img_height: int,
                    node_name: str,
                    subscriber_name: str):
        

        self.img_width, self.img_height = img_width, img_height
        
        self.focus_x = -670
        self.focus_y = -670
        self.x_center = int(0.5 * self.img_width)
        self.y_center = int(0.5 * self.img_height)
                
        self.node_name = node_name
        self.subscriber_name = subscriber_name
        self.bridge = CvBridge()        
        self.sub = rospy.Subscriber(name = self.subscriber_name,
                                        data_class = Image,
                                        callback = self.img_msg_to_cv,
                                        queue_size=1)


        self.cv_img = np.zeros((img_height, img_width, 3), dtype= np.uint8)
        self.blurred_img = cv2.GaussianBlur( self.cv_img, (5,5), 0)
        self.cv_img_hsv = cv2.cvtColor(self.blurred_img, cv2.COLOR_BGR2HSV)

        self.mask_red = np.zeros((img_height, img_width, 3), dtype= np.uint8)
        self.cv_img_masked_red = np.zeros((img_height, img_width, 3), dtype= np.uint8)
        
        self.mask_yellow = np.zeros((img_height, img_width, 3), dtype= np.uint8)
        self.cv_img_masked_yellow = np.zeros((img_height, img_width, 3), dtype= np.uint8)
        
        self.i_see_fire: bool = False
        self.max_fire_area = None

        self.max_countour = None
        self.detected = False

        self.seq = 0
        

    def get_camera_dimensions(self) -> tuple:
        return self.img_width, self.img_height

    def get_camera_centers(self) -> tuple:
        return self.x_center, self.y_center

    def img_msg_to_cv(self, img_msg: Image) -> None:
        
        cv_img_msg = self.bridge.imgmsg_to_cv2(img_msg, 'passthrough')

        cv_img_msg = cv2.resize(cv_img_msg, (self.img_width, self.img_height))

        self.cv_img = cv2.cvtColor(cv_img_msg, cv2.COLOR_BGR2RGB)

        self.detected = False
        
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

    def find_centroid(self) -> float or None:
        max_area = None
        
        countours, hierarchy = cv2.findContours(self.mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours( self.mask_red, countours, 0, (0,255,0), 3)
        
        for countour in countours: 
            self.red_area = cv2.contourArea(countour)
            #print(self.area)
            if self.red_area>10000: 

                if max_area:
                    if self.red_area > max_area:
                        max_area = self.red_area
                        self.max_countour = countour
                else:
                    max_area = self.red_area
                    self.max_countour = countour
                
                self.red_moment = cv2.moments(countour)
                # position of the centroid
                self.red_cx = int(self.red_moment["m10"]/self.red_moment["m00"]) 
                self.red_cy = int(self.red_moment["m01"]/self.red_moment["m00"])
                #  Creating a point to represent the centroid 
                cv2.circle( self.cv_img, (self.red_cx, self.red_cy), 7, (255, 255, 255), -1)

        return max_area    

    def find_dimensions_of_fire(self) -> tuple:
        x_min, y_min, x_max, y_max = helper.bounding_box_vertices(self.mask_red)

        cv2.rectangle(self.mask_red, (x_min, y_min), (x_max, y_max), (255, 0, 0), 3)
        cv2.circle(self.mask_red, ((x_max + x_min) // 2, (y_min + y_max) // 2), 7, (0, 0, 0), -1)

        return x_min, y_min, x_max, y_max

    def estimate_3d_coordinates(    self,
                                    x_pixel: np.ndarray,
                                    y_pixel: np.ndarray,
                                    z_gps: float) -> np.ndarray:

        '''
            https://mayavan95.medium.com/3d-position-estimation-of-a-known-object-using-a-single-camera-7a82b37b326b

            X: x Position in real world(**) of a pixel located at (x_pixel, y_pixel)
            Y: y Position in real world of a pixel located at (x_pixel, y_pixel)
            Z: height of the uav

            #* (**): em relacao ao referencial do drone.
        ''' 
        fx = self.focus_x
        fy = self.focus_y

        cx = int(0.5*self.img_width)
        cy = int(0.5*self.img_height)

        Z =  z_gps #altura em metros
        X = Z * (x_pixel - cx)/fx
        Y = Z * (y_pixel - cy)/fy
        return X, Y

    def display_img(self, view_red: bool = False, view_yellow: bool = False):
        cv2.imshow("UAV View", self.cv_img)

        if view_red:
            cv2.imshow('Mask Red', self.mask_red)

        if view_yellow:
            cv2.imshow('Mask Yellow', self.mask_yellow)

        k = cv2.waitKey(33)
        
        

    def update_state(self) -> None:        
        
        if not self.detected:
            self.color_detection(img_src = self.cv_img)
            self.max_fire_area = self.find_centroid()

            self.detected = True
