#!/usr/bin/python3
import numpy as np
from re import A
from matplotlib.pyplot import connect
import rospy
import os
from threading import Thread
from multiprocessing import Process
import time
import json
import traceback
import logging
#import random
import sys
import cv2

from tkinter import CENTER
from uav import UAV
import helper

from mavros_msgs import srv
from mrs_msgs.srv import String

STARTING = 0
STARTED = 1
GOING_TO_START = 2
PATROLLING = 3
GOING_TO_FIRE = 4
FIRE_CENTRALIZING = 5
FIRE_CENTRALIZED = 6

pi = np.pi

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

        self.curr_formation_name = ''
        self.curr_formation_coords = np.empty((0,4))

        self.curr_formation_pose = np.array([0,0,0])
        self.des_formation_pose = np.array([0,0,0])

        self.uavs = None

        rospy.init_node(name = node_name)
      
        # Talvez de para tirar essa parte do código
        self.op_num = 0
        self.formation_list = {}


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

        #for uav, position in zip(self.uavs, self.formation):
        for uav, position in zip(self.uavs, self.des_formation_coords):
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


    def land_all_there(self, position) -> None:
        for uav in self.uavs:
            uav.land_position(position)

    # Formations

    # Formations
    def setFormation(self, shape, N, L) -> None:
        if (shape=='line'):
            coord = self.line(N, L)
        elif (shape=='circle'):
            coord = self.circle(N, L)
        else:
            raise Exception('Formation input doesn\'t match any built-in formations')

        self.des_formation_coords = coord

        # Translate formation for current formation pose
        tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
        self.des_formation_coords = helper.translateFormation(self.des_formation_coords, tx, ty, tz)

        # Update formation name
        self.des_formation_name = shape
        #self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
        #self.op_num += 1

    def applyFormation(self) -> None:
        
        for uav, position in zip(self.uavs, self.des_formation_coords):
            uav.go_to_point(position)
            
        self.curr_formation_coords = self.des_formation_coords
    
    def line(N, L=1):
        if L/N < 1:
            L = N-1
            print("Distance between drones is too short\nSetting new length as {}".format(L))
        coord = np.empty((0,4))
        z0 = 30
        f = L/(N-1)
        logging.debug("Beginning line formation")
        for idx in range(N):
            point = [round(f*(N-1-idx),2), 0, z0, 1]
            coord = np.concatenate((coord,[point]))
        coord = helper.translateFormation(coord, -L/2, 0, 0)
        logging.debug("Line done\n")
        return coord   

    def circle(self, N, L):  # L é o raio e N é o numero de drones
        xc = yc = 0
        if 2*pi*L < N:
            L = round(N/(2*pi),2)
            print("Distance between drones is too short\nSetting new length as {}".format(L))
        coord = np.empty((0,4))
        z0 = 30
        logging.debug("Beginning circle formation")
        angle = 2*pi/N
        for idx in range(N):    
            xi = L*np.cos(idx*angle)
            yi = L*np.sin(idx*angle)
            point = [round(xc+xi,2), round(yc+yi,2), 30, 1]
            coord = np.concatenate((coord,[point]))
        logging.debug("Circle done\n")
        return coord


    # Operations with the swarm formations

    def translateFormation(self, position) -> None:
    
        tx = position[0]
        ty = position[1]
        tz = position[2]
        # Translate formation
        self.des_formation_coords = helper.translateFormation(self.des_formation_coords, tx, ty, tz)
        # Update formation pose 
        self.des_formation_pose = np.array([self.des_formation_pose[0]+tx, self.des_formation_pose[1]+ty, self.des_formation_pose[2]+tz])
        # Update formation name
        self.des_formation_name = 'translate'
        # Append to formation list
        #self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
        #self.op_num += 1
    

    def rotateFormation(self, anglex_deg, angley_deg, anglez_deg):
    
        anglex_rad = anglex_deg*np.pi/180
        angley_rad = angley_deg*np.pi/180
        anglez_rad = anglez_deg*np.pi/180
        # Get x, y, z of current formation
        tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
        # Translate back to the origin 
        origin_coords = helper.translateFormation(self.des_formation_coords, -tx, -ty, -tz)
        # Rotate formation on the origin
        origin_coords = helper.rotateFormation(origin_coords, anglex_rad, angley_rad, anglez_rad)
        # Translate back to the current pose
        self.des_formation_coords = helper.translateFormation(origin_coords, tx, ty, tz)
        # Update formation pose (stays the same in this case)
        self.des_formation_pose = np.array([self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]])
        # Update formation name
        self.des_formation_name = 'rotate'
        # Append to formation list
        #self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
        #self.op_num += 1
    
    def scaleFormation(self, sx, sy, sz):
        # Get x, y, z of current formation
        tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
        # Translate back to the origin 
        origin_coords = helper.translateFormation(self.des_formation_coords, -tx, -ty, -tz)
        # Scale formation
        origin_coords = helper.scaleFormation(origin_coords, sx, sy, sz)
        # Translate back to the current pose
        self.des_formation_coords = helper.translateFormation(origin_coords, tx, ty, tz)
        # Update formation pose (stays the same in this case)
        self.des_formation_pose = np.array([self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]])
        self.des_formation_name = 'scale'
        # Append to formation list
        #self.formation_list['formation {}'.format(self.op_num)] = {'name':self.des_formation_name, 'coord':self.des_formation_coords}
        #self.op_num += 1
    
