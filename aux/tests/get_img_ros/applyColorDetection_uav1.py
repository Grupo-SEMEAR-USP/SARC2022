from sensor_msgs.msg import Image
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np


bridge = CvBridge()

def apply_ColorDetection(img):
  blurred_img = cv2.GaussianBlur(img, (5,5), 0)

  hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV)
  
  lower_hsv= np.array([0, 118, 24])
  upper_hsv = np.array([27, 206, 116])
  
  mask_hsv = cv2.inRange(hsv, (lower_hsv), (upper_hsv))
  result_hsv = cv2.bitwise_and(img, img, mask=mask_hsv)
  
  show_image(result_hsv, "UAV 1 Mask View")
  
  countours, hierarchy= cv2.findContours(mask_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  cv2.drawContours(img, countours, -1, (0,255,0), 3)
  
  return img

def show_image(img, text):
  cv2.imshow(text, img)
  k = cv2.waitKey(33)

def image_callback(img_msg):
  
  #rospy.loginfo(img_msg.header)

  try:
    cv_image = bridge.imgmsg_to_cv2(img_msg, 'passthrough')
  except CvBridgeError as e:
    rospy.logerr("CvBridge Error: {}".format(e))
    
  cv_image_resized = cv2.resize(cv_image, (640, 360))
    
  #corrected_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
  corrected_img = cv2.cvtColor(cv_image_resized, cv2.COLOR_BGR2RGB)
  
  filter_img = apply_ColorDetection(np.copy(corrected_img))
  
  #show_image(cv_image_resized, uav_id)
  show_image(corrected_img, "UAV 1 View")
  show_image(filter_img, "UAV 1 Filtered View")



def main():
  rospy.init_node('uavs_imgs', anonymous=True)

  rospy.loginfo("Hello ROS!")

  cv2.namedWindow("UAV 1 View")
  cv2.namedWindow("UAV 1 Mask View")
  cv2.namedWindow("UAV 1 Filtered View")
  
  uav1_img = rospy.Subscriber('/uav1/rgbd_down/color/image_raw', Image, image_callback)

  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  main()