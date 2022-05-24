#!/usr/bin/env python3
import numpy as np
import rospy
from sensors.camera import uavCamera
from sensors.gps import uavGPS


'''
    A classe UAV deve facilitar o o gerenciamento dos sistemas que compoe cada 
    drone (visao, gps, etc). Tambem deve permitir construir o swarm de forma 
    mais facil.

'''
class UAV:

    def __init__(self, node_name: str):
        
        self.node_name = node_name
        self.camera = uavCamera(img_width = 720, img_height = 640,
                                node_name = 'uavs_imgs',
                                subscriber_name = '/uav1/rgbd_down/color/image_raw')


                
        self.gps = uavGPS(  node_name = 'odom_msgs',
                            subscriber_name = '/uav1/odometry/odom_gps')

        
        rospy.init_node(name = node_name)

    def update_state(self) -> None:
        
        # self.camera.update_state()
        self.gps.update_state()
        print('GPS data: ', self.gps.odometry_msg)
        
        if not rospy.is_shutdown():
            rospy.spin()
        

#uav de test
meu_uav = UAV(node_name = 'meu_uav')

if __name__ == '__main__':
    meu_uav.update_state()