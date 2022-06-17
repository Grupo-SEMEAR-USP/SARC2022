from sensor_msgs.msg import Image
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np


bridge = CvBridge()


def image_callback(img_msg):
    
    cv_img_msg = bridge.imgmsg_to_cv2(img_msg, 'passthrough')

    cv_img_msg = cv2.resize(cv_img_msg, (720, 640))

    cv_img = cv2.cvtColor(cv_img_msg, cv2.COLOR_BGR2RGB)

    cv2.imshow("UAV 1 view", cv_img)

    k = cv2.waitKey(33)


def main():
    rospy.init_node('uav_img', anonymous=True)
    
    rospy.loginfo("Hello ROS!")

    sub = rospy.Subscriber('/uav1/rgbd_down/color/image_raw', Image, image_callback)
    
    while not rospy.is_shutdown():
        rospy.spin()
    
    



if __name__ == '__main__':
    main()