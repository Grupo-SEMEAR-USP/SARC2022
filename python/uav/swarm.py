import cv2
import numpy as np
from uav import UAV
import rospy

class Swarm:

    def __init__(self, swarm_size: int, uavs: tuple, node_name: str):
        
        '''
            -> swarm_size: number of uavs in the swarm
            -> uavs: tuple of UAV objects (ex: (uav1, uav2, ...))
            -> who_detected_fire: list of len = len(uavs), filled with boolean 
            values. Ex: [True, False, False, ..] -> just uavs[0] detected fire.

        '''
        
        self.node_name = node_name
        self.swarm_size = swarm_size
        self.uavs = uavs
        self.uavs_arr = np.array(uavs)
        
        for uav in uavs:
            uav.configure()
        
        self.who_detected_fire: list = []

        rospy.init_node(name = node_name)
        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()

