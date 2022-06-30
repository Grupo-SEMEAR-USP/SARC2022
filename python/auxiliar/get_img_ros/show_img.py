from sensor_msgs.msg import Image
import rospy
import sys

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np


bridge = CvBridge()


def image_callback(img_msg, id):
    
    cv_img_msg = bridge.imgmsg_to_cv2(img_msg, 'passthrough')

    cv_img_msg = cv2.resize(cv_img_msg, (720, 480))

    cv_img = cv2.cvtColor(cv_img_msg, cv2.COLOR_BGR2RGB)

    cv2.imshow(f"UAV {id} view", cv_img)

    k = cv2.waitKey(33)


def main():
    uav_id = sys.argv[1] if len(sys.argv) > 1 else 1

    rospy.init_node('uav_img', anonymous=True)
    
    rospy.loginfo(f"Viewing UAV {uav_id}")

    sub = rospy.Subscriber(f'/uav{uav_id}/rgbd_down/color/image_raw', Image, image_callback, uav_id)
    
    while not rospy.is_shutdown():
        rospy.spin()
    
    



if __name__ == '__main__':
    main()