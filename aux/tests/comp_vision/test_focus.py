from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import rospy
import sys

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np

FOCUS_X = -540
FOCUS_Y = -620

CENTER_X = 360
CENTER_Y = 240

def nothing(x):
    pass

class FIND:
    bridge = CvBridge()

    def __init__(self, uav_id):
        self.stated = False

        self.id = uav_id

        self.x = self.y = 0

        self.num_media = 1

        rospy.init_node('uav_img', anonymous=True)
    
        rospy.loginfo(f"Viewing UAV {uav_id}")

        self.camera = rospy.Subscriber(f'/uav{uav_id}/rgbd_down/color/image_raw', Image, self.image_callback)
        self.gps = rospy.Subscriber(f'/uav{uav_id}/odometry/odom_gps', Odometry, self.gps_callback)


    def gps_callback(self, odom_msg: Odometry):
        position = odom_msg.pose.pose.position
        self.pos_x = position.x
        self.pos_y = position.y
        self.pos_z = position.z

    def image_callback(self, img_msg):
        cv_img_msg = self.bridge.imgmsg_to_cv2(img_msg, 'passthrough')

        cv_img_msg = cv2.resize(cv_img_msg, (720, 480))

        self.cv_img = cv2.cvtColor(cv_img_msg, cv2.COLOR_BGR2RGB)

        _img = np.copy(self.cv_img)

        _blurred_img = cv2.GaussianBlur( _img, (5,5), 0)

        self.img = cv2.cvtColor(_blurred_img, cv2.COLOR_BGR2HSV)
        self.stated = True

    def test_focus(self, pixel_x, pixel_y):

        if self.num_media == 1:
            self.x = self.pos_z * (pixel_x - CENTER_X) / FOCUS_X
            self.y = self.pos_z * (pixel_y - CENTER_Y) / FOCUS_Y
        else:
            self.x *= (self.num_media - 1)
            self.y *= (self.num_media - 1)

            self.x += self.pos_z * (pixel_x - CENTER_X) / FOCUS_X
            self.y += self.pos_z * (pixel_y - CENTER_Y) / FOCUS_Y

            self.x /= self.num_media
            self.y /= self.num_media

    def show(self):

        mask = cv2.inRange(self.img, np.array([0, 0, 0]), np.array([40, 40, 255]))

        countours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(countours) > 0:
            moment = cv2.moments(countours[0])
            # position of the centroid
            try:
                cx = int(moment["m10"]/moment["m00"]) 
                cy = int(moment["m01"]/moment["m00"])
            except:
                return
            #  Creating a point to represent the centroid 
            cv2.circle( mask, (cx, cy), 7, (0, 0, 0), -1)

            #print(f'x:{cx} y:{cy}')

            self.test_focus(cx, cy)

            self.num_media += 1

            print(f'\npos_x: {self.pos_x + self.y}\npos_y: {self.pos_y + self.x}')

        cv2.imshow(f"UAV {self.id} view", self.cv_img)
        cv2.imshow(f"UAV {self.id} mask", mask)

        k = cv2.waitKey(1) & 0xff



def main():
    uav_id = sys.argv[1] if len(sys.argv) > 1 else 1

    find = FIND(uav_id)
    
    while not rospy.is_shutdown():
        if find.stated:
            find.show()
    
        pass

    cv2.destroyAllWindows()
    



if __name__ == '__main__':
    main()
