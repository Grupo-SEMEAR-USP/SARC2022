from tkinter import CENTER
from uav import UAV

import numpy as np
import rospy
import os

from mrs_msgs.srv import String

STARTING = 0
STARTED = 1
GOING_TO_START = 2
PATROLLING = 3
FIRE_FINDED = 4
GOING_TO_FIRE = 5
FIRE_CENTRALIZING = 6
FIRE_CENTRALIZED = 7

class Swarm:

    def __init__(self, swarm_size: int, node_name: str, center_drone: int = 1, spawn: bool = False):
        
        '''
            -> swarm_size: number of uavs in the swarm
            -> uavs: tuple of UAV objects (ex: (uav1, uav2, ...))
            -> who_detected_fire: list of len = len(uavs), filled with boolean 
            values. Ex: [True, False, False, ..] -> just uavs[0] detected fire.

        '''

        self.node_name = node_name
        self.swarm_size = swarm_size
        self.center_drone = center_drone

        self.state = STARTING
        self.formation = []
        self.trajectories = {}

        self.uavs = None

        rospy.init_node(name = node_name)

        if center_drone < 1 or center_drone > swarm_size:
            rospy.logerr('Drone central incorreto!')
            return 

        if spawn:
            self.spawn_drones()

        self.uavs = (UAV(i, start_init=False) for i in range(1, swarm_size+1))
        
        for uav in self.uavs:
            uav.configure()
            self.trajectories[uav.node_name] = []

        rospy.loginfo("UAVs Configured")

        rospy.sleep(15)

        rospy.loginfo("Going to Start Formation")

        self.state = STARTED

        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()

        self.create_start_formation()
        self.goto_formation()


    def update(self):
        
        '''if self.state == STARTED:
            self.create_start_formation()
            self.goto_formation()
            self.state = GOING_TO_START
            
            rospy.loginfo("Going to Start Position")

        elif self.state == GOING_TO_START:
            if self.is_on_formation():
                rospy.sleep(5)
                self.create_patrolling_formation()
                self.goto_formation()
                self.state = PATROLLING

                rospy.loginfo("Starting Patrolling...")

        elif self.state == PATROLLING:
            pass'''
        pass

    def create_start_formation(self) -> None:

        self.formation = []

        dAngle = np.pi * 2 / (self.swarm_size - 1)
        angle = 0

        for i in range(self.swarm_size):

            if i == self.center_drone:
                x = y = 0
            else:
                x = int(10 * np.cos(angle) * 100) / 100
                y = int(10 * np.sin(angle) * 100) / 100

                angle += dAngle

            self.formation.append([x, y, 30, 0.0])

    def create_patrolling_trajectory(self) -> None:

        self.trajectory = {}

        position_in_quadrant = [(9, 9),
                                (9, 57),
                                (57, 57),
                                (57, 41),
                                (25, 41),
                                (25, 25),
                                (57, 25),
                                (57, 9),
                                (9, 9)]

        for i, uav in enumerate(self.uavs):
            id_x = i // 3
            id_y = i % 3

            x = -100 + id_x * 66
            y = -100 + id_y * 66

            self.trajectories[uav.node_name] = []

            for position in position_in_quadrant:
                pos_x = x + position[0]
                pos_y = y + position[1]
                self.trajectories[uav.node_name].append([pos_x, pos_y, 30, 0.0])

    def anyone_find_fire(self) -> bool:
        
        for uav in self.uavs:
            pass

    def goto_formation(self) -> None:

        for uav, position in zip(self.uavs, self.formation):
            uav.go_to_point(position)

    def is_on_formation(self) -> bool:

        for uav, position in zip(self.uavs, self.formation):
            if not uav.is_on_point(position): 
                return False
        
        return True

    def spawn_drones(self) -> None:
        uav_type = os.environ.get('UAV_TYPE')
        camera = os.environ.get('CAMERA')

        rospy.wait_for_service(f'/mrs_drone_spawner/spawn', timeout = None)
        self.spawner = rospy.ServiceProxy(f'/mrs_drone_spawner/spawn', String)

        dAngle = np.pi * 2 / (self.swarm_size - 1)
        angle = 0

        for i in range(1, self.swarm_size+1):
            if i == self.center_drone:
                x = y = 0
            else:
                x = int(10 * np.cos(angle) * 100) / 100
                y = int(10 * np.sin(angle) * 100) / 100

                angle += dAngle

            res = self.spawner(f"{i} {uav_type} --enable-rangefinder --enable-ground-truth --{camera} --pos {x} {y} 0.5 0.0")

            #print(f'Uav{i} Spawn ({x} , {y}) = {res.success} => {res.message}')
            rospy.loginfo(f'Uav{i} Spawn = {res.success} => {res.message}')
