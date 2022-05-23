import numpy as np
import rospy
from camera.camera import uavCamera
from gps.gps import uavGPS


'''
    A classe UAV deve facilitar o o gerenciamento dos sistemas que compoe cada 
    drone (visao, gps, etc). Tambem deve permitir construir o swarm de forma 
    mais facil.

'''
class UAV:

    def __init__(self):
        
        self.camera = uavCamera()
                
        self.gps = uavGPS()
        self.xyz_position = (0, 0, 0)