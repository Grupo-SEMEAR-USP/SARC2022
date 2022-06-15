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
GOING_TO_FIRE = 4
FIRE_CENTRALIZING = 5
FIRE_CENTRALIZED = 6

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

        self.uavs = tuple(UAV(i, start_init=False) for i in range(1, swarm_size+1))
        
        for uav in self.uavs:
            uav.configure()
            self.trajectories[uav.node_name] = []

        rospy.loginfo("UAVs Configured")

        self.countdown()

        self.state = STARTED

        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()

    def countdown(self) -> None:
        rospy.sleep(1)
        rospy.loginfo("5...")
        rospy.sleep(0.5)
        rospy.loginfo("4...")
        rospy.sleep(0.5)
        rospy.loginfo("3...")
        rospy.sleep(0.5)
        rospy.loginfo("2...")
        rospy.sleep(0.5)
        rospy.loginfo("1...")
        rospy.sleep(0.5)

    def update(self):

        for uav in self.uavs:
            uav.update_state()
        
        if self.state == STARTED:
            self.create_start_formation()
            self.goto_formation()
            self.state = GOING_TO_START
            
            rospy.loginfo("Going to Start Position")

        elif self.state == GOING_TO_START:
            if self.is_on_formation():
                rospy.loginfo('First formation reached')
                self.countdown()
                self.create_patrolling_trajectory()
                self.start_trajectory()
                self.state = PATROLLING

                rospy.loginfo("Starting Patrolling...")

        elif self.state == PATROLLING:
            uav_that_found = self.anyone_found_fire()
            if uav_that_found:
                rospy.loginfo('Fire Founded')
                self.stop_trajectory()
                self.create_circle_formation_around_uav(uav_that_found, 30, 30)
                self.goto_formation()

                self.state = GOING_TO_FIRE
        elif self.state == GOING_TO_FIRE:
            if self.is_on_formation():
                rospy.loginfo('Fire initial formation reached')

                self.state = FIRE_CENTRALIZING
        
        elif self.state == FIRE_CENTRALIZING:
            self.centralize_on_fire()

    def centralize_on_fire(self) -> None:

        uav = self.uavs[self.center_drone]

        enclosed = uav.check_enclosing_circle()

    def create_start_formation(self) -> None:
        
        self.create_circle_formation(0, 0, 30)

    def create_circle_formation_around_uav(self, uav_centered: UAV, altitude: float, centered_altitude: float) -> None:

        pos_x = uav_centered.pos_x
        pos_y = uav_centered.pos_y

        self.create_circle_formation(pos_x, pos_y, altitude, centered_altitude)

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

        quadrant_distribution = {1: (1, 1),
                                 2: (2, 1),
                                 3: (2, 2),
                                 4: (1, 2),
                                 5: (0, 2),
                                 6: (0, 1),
                                 7: (0, 0),
                                 8: (1, 0),
                                 9: (2, 0)}

        for i in range(self.swarm_size):

            uav = self.uavs[i]

            quadrant = quadrant_distribution[uav.uav_id]

            id_x, id_y = quadrant

            x = -100 + id_x * 66
            y = -100 + id_y * 66

            self.trajectories[uav.node_name] = []

            for position in position_in_quadrant:
                pos_x = x + position[0]
                pos_y = y + position[1]
                self.trajectories[uav.node_name].append([pos_x, pos_y, 30, 0.0])

    def create_circle_formation(self, center_x: float, center_y: float, altitude: float, centered_altitude: float = -1) -> None:
        
        self.formation = []

        dAngle = np.pi * 2 / (self.swarm_size - 1) if self.swarm_size > 1 else 0
        angle = 0

        for i in range(1, self.swarm_size+1):

            if i == self.center_drone:
                x = center_x
                y = center_y
                z = centered_altitude if centered_altitude > 0 else altitude
            else:
                x = center_x + int(10 * np.cos(angle) * 100) / 100
                y = center_y + int(10 * np.sin(angle) * 100) / 100
                z = altitude

                angle += dAngle

            self.formation.append([x, y, z, 0.0])

    def start_trajectory(self) -> None:

        failed_spawns = []

        for uav in self.uavs:
            res_1 = uav.trajectory_generation(self.trajectories[uav.node_name], 1)
            res_2 = uav.start_trajectory()

            if not res_1.success or not res_2.success:
                failed_spawns.append(uav.uav_id)

        if not failed_spawns:
            rospy.loginfo(f'All UAVs started trajectory')
        else:
            uavs = failed_spawns if len(failed_spawns) == 1 else failed_spawns.join(', ') 
            rospy.loginfo(f'UAVs {uavs} did not start trajectory')

    def stop_trajectory(self) -> None:
        failed_spawns = []

        for uav in self.uavs:
            res = uav.stop_trajectory()

            if not res.success:
                failed_spawns.append(uav.uav_id)

        if not failed_spawns:
            rospy.loginfo(f'All UAVs stoped moving')
        else:
            uavs = failed_spawns if len(failed_spawns) == 1 else failed_spawns.join(', ') 
            rospy.loginfo(f'UAVs {uavs} did not stop moving')

    def anyone_found_fire(self) -> None or UAV:

        uav_that_finded = None
        
        for uav in self.uavs:
            if uav.did_i_detect_fire:
                uav_that_finded = uav
                break

        return uav_that_finded

    def goto_formation(self) -> None:

        failed_spawns = []

        for i in range(self.swarm_size):
            uav = self.uavs[i]
            position = self.formation[i]

            res = uav.go_to_point(position)

            if not res.success:
                failed_spawns.append(uav.uav_id)

        if not failed_spawns:
            rospy.loginfo(f'All UAVs sent to position')
        else:
            uavs = failed_spawns if len(failed_spawns) == 1 else failed_spawns.join(', ') 
            rospy.loginfo(f'UAVs {uavs} did not sent to position')

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

        dAngle = np.pi * 2 / (self.swarm_size - 1) if self.swarm_size > 1 else 0
        angle = 0

        failed_spawns = []

        for i in range(1, self.swarm_size+1):
            if i == self.center_drone:
                x = y = 0
            else:
                x = int(10 * np.cos(angle) * 100) / 100
                y = int(10 * np.sin(angle) * 100) / 100

                angle += dAngle

            res = self.spawner(f"{i} {uav_type} --enable-rangefinder --enable-ground-truth --{camera} --pos {x} {y} 0.5 0.0")

            if not res.success:
                failed_spawns.append(i)

        if not failed_spawns:
            rospy.loginfo(f'All UAVs spawns succeeded')
        else:
            uavs = failed_spawns if len(failed_spawns) == 1 else failed_spawns.join(', ') 
            rospy.loginfo(f'UAVs {uavs} not spawned successfully')