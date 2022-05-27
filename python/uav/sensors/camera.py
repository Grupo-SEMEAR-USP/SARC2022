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
        self.cv_img_hsv = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
        self.i_see_fire: bool = False

        #Init camera node
        # rospy.init_node(node_name, anonymous = True)
        # rospy.loginfo("Camera started")

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
        #Subscribe to camera in Gazebo and apply img_msg_to_cv() as callback function
        # self.img_msg = rospy.Subscriber(name = self.subscriber_name,
        #                                 data_class = Image,
        #                                 callback = self.img_msg_to_cv)

        #Resize image
        self.cv_img = cv2.resize(self.cv_img, (self.img_height, self.img_width))
        
        # self.cv_img_hsv = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
        

    def color_detection(    self,
                            img_src: np.array,
                            lower_values: list,
                            upper_values: list) -> None:
        
        __img = np.copy(img_src)

        self.mask = cv2.inRange(    __img,
                                    lower_values,
                                    upper_values)

        self.cv_img_masked = cv2.bitwise_and(   __img,
                                                __img,
                                                mask=self.mask)                    
        
    def display_img(self):
        cv2.imshow("uav View", self.cv_img)
        
        

    def update_state(self) -> None:        
        self.sub = rospy.Subscriber(name = self.subscriber_name,
                                        data_class = Image,
                                        callback = self.img_msg_to_cv)

        

        
        
        
