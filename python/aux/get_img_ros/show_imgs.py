from sensor_msgs.msg import Image
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np


bridge = CvBridge()

bigImage = np.zeros((490, 980, 3), np.uint8)

def show_image(img, id):
	cv2.imshow("UAV {}".format(id), img)
	k = cv2.waitKey(33)
 
def show_big_image(img):
  cv2.imshow("UAVs", img)
  k = cv2.waitKey(33)

def image_callback(img_msg, uav_id):
	
	#rospy.loginfo(img_msg.header)

	try:
		cv_image = bridge.imgmsg_to_cv2(img_msg, 'passthrough')
	except CvBridgeError as e:
		rospy.logerr("CvBridge Error: {}".format(e))

	cv_image_resized = cv2.resize(cv_image, (320, 240))

	if uav_id == 1:
		bigImage[0:240, 0:320, :] = cv_image_resized
	elif uav_id == 2:
		bigImage[0:240, 330:650, :] = cv_image_resized
	elif uav_id == 3:
		bigImage[0:240, 660:980, :] = cv_image_resized
	elif uav_id == 4:
		bigImage[250:490, 113:433, :] = cv_image_resized
	else:
		bigImage[250:490, 547:867, :] = cv_image_resized
  
	#show_image(cv_image_resized, uav_id)
	show_big_image(bigImage)


def main():
	num_drones = 5
	
	print("Hello")
	
	rospy.init_node('uavs_imgs', anonymous=True)
	
	rospy.loginfo("Hello ROS!")
	
	cv2.namedWindow("UAVs")
 
	imgs = []
	
	for i in range(1, num_drones+1):
		imgs.append(rospy.Subscriber('/uav{}/rgbd_down/color/image_raw'.format(i), Image, image_callback, i))
		#cv2.namedWindow("UAV {}".format(i), i)
	
	while not rospy.is_shutdown():
		rospy.spin()
	
	



if __name__ == '__main__':
	main()