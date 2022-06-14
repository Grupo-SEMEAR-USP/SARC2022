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
        
        #https://mayavan95.medium.com/3d-position-estimation-of-a
        #-known-object-using-a-single-camera-7a82b37b326b
        #TODO: pegar os valores intrisics no arquivo .xacro da camera
        self.focus_x = 277
        self.focus_y = 277
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
        
        self.i_see_fire: bool = False
        self.max_fire_area = None

        self.seq = 0
        

    # img(ROS) -> img(OpenCV)
    def img_msg_to_cv(self, img_msg: Image) -> None:
        
        # msg.Image -> 'cv2.Image'
        cv_img_msg = self.bridge.imgmsg_to_cv2(img_msg, 'passthrough')

        #Color correction
        self.cv_img = cv2.cvtColor(cv_img_msg, cv2.COLOR_BGR2RGB)

        #cv2.imwrite(f'images/img{img_msg.header.seq}.jpg', self.cv_img)

        #self.color_detection(img_src = self.cv_img)
        #self.max_fire_area = self.find_centroid()

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
    def find_centroid(self) -> float or None:
        max_area = None
        
        countours, hierarchy = cv2.findContours(self.mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours( self.mask_red, countours, 0, (0,255,0), 3)
        
        for countour in countours: 
            self.red_area = cv2.contourArea(countour)
            #print(self.area)
            if self.red_area>10000: 
                max_area = max(self.red_area, max_area) if max_area else self.red_area
                self.red_moment = cv2.moments(countour)
                # position of the centroid
                self.red_cx = int(self.red_moment["m10"]/self.red_moment["m00"]) 
                self.red_cy = int(self.red_moment["m01"]/self.red_moment["m00"])
                #  Creating a point to represent the centroid 
                cv2.circle( self.cv_img, (self.red_cx, self.red_cy), 7, (255, 255, 255), -1)

        return max_area    


    @staticmethod
    def bounding_box_vertices(img_mask: np.array) -> tuple:
   
        '''
            Bounding box of detected region (ex: red area)
        '''
        xys_pts = np.nonzero(img_mask)
        x_of_interest = xys_pts[1]
        y_of_interest = xys_pts[0]
        
        xmin, xmax = min(x_of_interest), max(x_of_interest)
        ymin, ymax = min(y_of_interest), max(y_of_interest)
        
        ##Vertices/corners:
        # up_left, up_right = (xmin, ymin), (xmax, ymin)
        # down_left, down_right = (xmin, ymax), (xmax, ymax)

        ##Draw the rectangle 
        # img_temp = np.copy(img_mask)
        # cv2.rectangle(img_temp, (xmin, ymin), (xmax, ymax), (255, 0, 0), 1)
        
        return xmin, ymin, xmax, ymax

    
    
    def estimate_3d_coordinates(    self,
                                    x_pixel: np.array,
                                    y_pixel: np.array,
                                    z_gps: float) -> np.array:

        '''
            #TODO: 
                1. Implementar uma funcao que recebe a posicao da odometria do
                drone (pos_x, pos_y) para passar X e Y para as coordenadas
                do mapa ao inves da do drone.
                2. Deixar o nome dessa funcao mais intuitivo


            https://mayavan95.medium.com/3d-position-estimation
            -of-a-known-object-using-a-single-camera-7a82b37b326b

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


    def display_img(self, all: bool = False):
        cv2.imshow("uav View", self.cv_img)
        if all:
            cv2.imshow("uav Mask Red", self.cv_img_masked_red)
            #cv2.imshow("uav Mask Yellow", self.cv_img_masked_yellow)

        k = cv2.waitKey(33)
        
        

    def update_state(self) -> None:        
        
        self.color_detection(img_src = self.cv_img)
        self.max_fire_area = self.find_centroid()
        
        #cv2.imwrite(f'images/img{self.seq}.jpg', self.cv_img)
        #self.seq += 1

