#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from sensors.camera import uavCamera
from sensors.gps import uavGPS


'''
    A classe UAV deve facilitar o o gerenciamento dos sistemas que compoe cada 
    drone (visao, gps, etc). Tambem deve permitir construir o swarm de forma 
    mais facil.

'''

#TODO:
'''
    1. Adicionar atributos importantes
        - posição do uav
        - i_see_fire = true/false
        - etc
'''

class UAV:

    def __init__(self, node_name: str):
        
        self.node_name = node_name
        self.camera = uavCamera(img_width = 720, img_height = 640,
                                node_name = 'uavs_imgs',
                                subscriber_name = '/uav1/rgbd_down/color/image_raw')


                
        self.gps = uavGPS(  node_name = 'odom_msgs',
                            subscriber_name = '/uav1/odometry/odom_gps')
        
        self.position = None
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None
        
        rospy.init_node(name = node_name)

    def update_state(self) -> None:
        
        self.camera.update_state()
        self.gps.update_state()

        self.position = self.gps.odometry_msg
        # self.pos_x = self.position.x
        # self.pos_y = self.position.y
        # self.pos_z = self.position.z

        rospy.loginfo("I heard: %s", self.position)
        
        

#uav de test
meu_uav = UAV(node_name = 'meu_uav')

if __name__ == '__main__':
    
    while not rospy.is_shutdown():
        meu_uav.update_state()
        
        meu_uav.camera.display_img()
        k = cv2.waitKey(1) & 0xff
        
    cv2.destroyAllWindows()
        