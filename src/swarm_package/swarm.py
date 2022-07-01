#!/usr/bin/python3

# Importing Objects
from swarm_package.uav import UAV

# Importing Python Files
from swarm_package import helper

# Import Libraries
import pandas as pd
import numpy as np
import rospy
import os

# Importing Ros Services
from mrs_msgs.srv import String


# Defining the state of the drones
STARTING = 0
STARTED = 1
GOING_TO_START = 2
PATROLLING = 3
GOING_TO_FIRE = 4
FIRE_CENTRALIZING = 5
GOINT_TO_FIRE_FORMATION = 6
FIRE_CENTRALIZED = 7
FIRE_FIGHTING = 8
GOING_TO_BASE = 9
FINISHED = 10

PI = np.pi

class Swarm:

    def __init__(self, swarm_size: int, node_name: str, spawn: bool = False):
        
        self.node_name = node_name
        self.swarm_size = swarm_size

        self.state = STARTING
        self.formation = []
        self.trajectories = {}

        self.curr_formation_coords = np.empty((0,4))

        self.curr_formation_pose = np.array([0,0,0])
        self.des_formation_pose = np.array([0,0,0])

        self.uavs = None

        self.last_sent_fire_position = None
        self.fire_centralizing_min_reduction = 0.3
        self.fire_centralizing_max_reduction = 0.75
        self.fire_centralizing_altitude_addition = 15
        self.fire_center_image_size_threshold = 20

        self.base_position_x =  113
        self.base_position_y = -121

        rospy.init_node(name = node_name)

        if spawn:
            self.spawn_drones()

        self.uavs = tuple(UAV(i, start_init=False) for i in range(1, swarm_size+1))
        
        for uav in self.uavs:
            uav.configure()
            self.trajectories[uav.node_name] = []

        rospy.loginfo("UAVs Configured")

        self.countdown()

        self.state = STARTED
        self.points_saved = False

        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()

    def countdown(self, time: int = 5) -> None:
        rospy.sleep(1)
        for t in range(time, 0, -1):
            rospy.loginfo(f"{t}...")
            rospy.sleep(0.5)

    def save_drones_travel_position(self) -> None:

        self.points_saved = True

        travels = {}
        time = []

        for i in range(self.swarm_size):
            uav = self.uavs[i]

            travel_points = uav.get_travel_points()

            travels[f'uav{uav.uav_id}_x'] = travel_points['x']
            travels[f'uav{uav.uav_id}_y'] = travel_points['y']
            travels[f'uav{uav.uav_id}_z'] = travel_points['z']

            if i == 0:
                time = travel_points['time']

        data_frame = pd.DataFrame(travels, index=time)
        data_frame.to_csv('data/travel_points.csv')

        rospy.loginfo("Drones Positions Saved!")

    def update(self):

        self.time_now = rospy.get_rostime()

        for uav in self.uavs:
            uav.update_state(self.t0.secs, self.time_now.secs)
        
        if self.state == STARTED:
            self.create_start_formation()
            self.applyFormation()
            rospy.loginfo('UAVs sent to first formation')
            self.state = GOING_TO_START

        elif self.state == GOING_TO_START:
            if self.is_on_formation():
                rospy.loginfo('First formation reached')
                self.create_patrolling_trajectory()
                self.start_trajectory()
                self.state = PATROLLING

                rospy.loginfo("Starting Patrolling...")

        elif self.state == PATROLLING:
            uav_that_found = self.anyone_found_fire()
            if uav_that_found:
                rospy.loginfo('Fire Founded')
                self.stop_trajectory()
                self.create_circle_formation_around_uav(uav_that_found, 10, 30, 40)
                self.applyFormation()

                self.state = GOING_TO_FIRE

        elif self.state == GOING_TO_FIRE:
            if self.is_on_formation():
                rospy.loginfo('Fire initial formation reached')

                self.state = FIRE_CENTRALIZING
        
        elif self.state == FIRE_CENTRALIZING:
            if self.centralize_on_fire():
                rospy.loginfo('Fire centralized')
                self.create_final_formation()
                self.applyFormation()

                self.state = GOINT_TO_FIRE_FORMATION

        elif self.state == GOINT_TO_FIRE_FORMATION:
            if self.is_on_formation():
                rospy.loginfo('Fire formation reached')

                self.state = FIRE_CENTRALIZED
        
        elif self.state == FIRE_CENTRALIZED:
            rospy.sleep(5)
            rospy.loginfo("Starting Fire Combat")
            self.fireCombat()
            self.state = FIRE_FIGHTING
        
        elif self.state == FIRE_FIGHTING:

            self.state = GOING_TO_BASE

        elif self.state == GOING_TO_BASE:
            rospy.loginfo("Returning to base")
            self.returnBase()
            self.state = FINISHED
                    

    def centralize_on_fire(self) -> bool:

        uav = self.uavs[0]

        if self.last_sent_fire_position and not uav.is_on_point_for_time(self.t0.secs, self.time_now.secs, 2, self.last_sent_fire_position):
            return

        x_min, y_min, x_max, y_max = uav.camera.find_dimensions_of_fire()

        center_pixel_x = (x_min + x_max) / 2
        center_pixel_y = (y_min + y_max) / 2

        camera_center_x, camera_center_y = uav.camera.get_camera_centers()

        is_on_center = helper.is_close_enough_2d(center_pixel_x, center_pixel_y, camera_center_x, camera_center_y, self.fire_center_image_size_threshold)

        if is_on_center:
            camera_width, camera_height = uav.camera.get_camera_dimensions()
            
            if x_min >= camera_width*0.1 and x_max <= camera_width*0.9 and y_min >= camera_height*0.1 and y_max <= camera_height*0.9:
                return True

            new_z = uav.pos_z+self.fire_centralizing_altitude_addition

            position = [uav.pos_x, uav.pos_y, new_z, 0.0]

            rospy.loginfo('Elevating to {:.0f} m'.format(new_z))
        else:
            real_x, real_y = uav.camera.estimate_3d_coordinates(center_pixel_x, center_pixel_y, uav.pos_z)

            dist = helper.distance_2d(center_pixel_x, center_pixel_y, camera_center_x, camera_center_y)
            max_dist = min(camera_center_x, camera_center_y)

            reduction = min(max(self.fire_centralizing_max_reduction * dist / max_dist,
                                self.fire_centralizing_min_reduction),
                            self.fire_centralizing_max_reduction)

            real_x *= reduction
            real_y *= reduction

            position = [uav.pos_x+real_y, uav.pos_y+real_x, uav.pos_z, 0.0]

        uav.go_to_point(position)
        self.last_sent_fire_position = position

        return False

    def create_final_formation(self) -> None:
        uav = self.uavs[0]

        x_min, y_min, x_max, y_max = uav.camera.find_dimensions_of_fire()

        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2

        if x_max - x_min > y_max - y_min:
            pos_x = x_max
            pos_y = center_y
        else:
            pos_x = center_x
            pos_y = y_max

        real_x, real_y = uav.camera.estimate_3d_coordinates(pos_x, pos_y, uav.pos_z)

        radius = helper.modulo_2d(real_x, real_y)
        self.radius_last = radius
        
        position = [uav.pos_x, uav.pos_y, 30]

        rospy.loginfo(f"Radius: {radius}")
        rospy.loginfo(f"Position: {(uav.pos_x, uav.pos_y)}")

        self.setFormation(radius, position, altitude_center=uav.pos_z)

    def create_start_formation(self) -> None:
        
        self.setFormation(10, [0, 0, 30])

    def create_circle_formation_around_uav(self, uav_centered: UAV, radius: float, altitude: float, centered_altitude: float) -> None:

        pos_x = uav_centered.pos_x
        pos_y = uav_centered.pos_y

        self.setFormation(radius, [pos_x, pos_y, altitude], centered_altitude)

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

    def start_trajectory(self) -> None:

        failed_uavs = []

        for uav in self.uavs:
            res_1 = uav.trajectory_generation(self.trajectories[uav.node_name], 1)
            res_2 = uav.start_trajectory()

            if not res_1.success or not res_2.success:
                failed_uavs.append(uav.uav_id)

        if not failed_uavs:
            rospy.loginfo(f'All UAVs started trajectory')
        else:
            rospy.loginfo(f'UAVs {failed_uavs} did not start trajectory')

    def stop_trajectory(self) -> None:
        failed_uavs = []

        for uav in self.uavs:
            res = uav.stop_trajectory()

            if not res.success:
                failed_uavs.append(uav.uav_id)

        if not failed_uavs:
            rospy.loginfo(f'All UAVs stoped moving')
        else:
            rospy.loginfo(f'UAVs {failed_uavs} did not stop moving')

    def anyone_found_fire(self) -> None or UAV:

        uav_that_found_fire = None
        
        for uav in self.uavs:
            if uav.did_i_detect_fire:
                uav_that_found_fire = uav
                break

        return uav_that_found_fire

    def is_on_formation(self) -> bool:
        self.time_now = rospy.get_rostime()

        for uav in self.uavs:
            uav.update_state(self.t0.secs, self.time_now.secs)

        for i, position in enumerate(self.des_formation_coords):
            uav = self.uavs[i]
            if not uav.is_on_point(1, self.t0.secs, self.time_now.secs, position):
                return False
        
        return True

    def spawn_drones(self) -> None:
        uav_type = os.environ.get('UAV_TYPE')
        camera = os.environ.get('CAMERA')

        rospy.wait_for_service(f'/mrs_drone_spawner/spawn', timeout = None)
        self.spawner = rospy.ServiceProxy(f'/mrs_drone_spawner/spawn', String)

        dAngle = np.pi * 2 / (self.swarm_size - 1) if self.swarm_size > 1 else 0
        angle = 0

        failed_uavs = []

        for i in range(1, self.swarm_size+1):
            if i == 1:
                x = y = 0
            else:
                x = int(10 * np.cos(angle) * 100) / 100
                y = int(10 * np.sin(angle) * 100) / 100

                angle += dAngle

            res = self.spawner(f"{i} {uav_type} --enable-rangefinder --enable-ground-truth --{camera} --pos {x} {y} 0.5 0.0")

            if not res.success:
                failed_uavs.append(i)

        if not failed_uavs:
            rospy.loginfo(f'All UAVs spawns succeeded')
        else:
            rospy.loginfo(f'UAVs {failed_uavs} not spawned successfully')


    def land_all(self) -> None:
        for uav in self.uavs:
            uav.land_now()

    # Formations
    def setFormation(self, radius: float, position: list, altitude_center: float = -1, start_angle: float = 0) -> None:

        coord = helper.circle(self.swarm_size-1, radius, start_angle)
        
        if altitude_center > 0:
            center_pos = [0, 0, altitude_center - position[2], 1]
        else:
            center_pos = [0, 0, 0, 1]

        coord = np.concatenate(([center_pos], coord))

        self.des_formation_coords = coord

        self.des_formation_pose = position

        # Translate formation for current formation pose
        tx, ty, tz = self.des_formation_pose[0], self.des_formation_pose[1], self.des_formation_pose[2]
        self.des_formation_coords = helper.translateFormation(self.des_formation_coords, tx, ty, tz)

    def applyFormation(self) -> None:

        for i in range(self.swarm_size):
            uav = self.uavs[i]
            position = []
            for j in range(len(self.des_formation_coords[i])):
                position.append(self.des_formation_coords[i][j])
            position[3] = 0
            uav.go_to_point(position)

    # Operations with the swarm formations

    def translateFormation(self, position: list) -> None:
    
        tx = position[0]
        ty = position[1]
        tz = position[2]
        # Translate formation
        self.des_formation_coords = helper.translateFormation(self.des_formation_coords, tx, ty, tz)
        # Update formation pose 
        self.des_formation_pose = np.array([self.des_formation_pose[0]+tx, self.des_formation_pose[1]+ty, self.des_formation_pose[2]+tz])
        

    def rotateFormation(self, anglex_deg: float, angley_deg: float, anglez_deg: float) -> None:
    
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
    
    def scaleFormation(self, sx: float, sy: float, sz: float) -> None:
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

        
    def fireCombat(self) -> None:

        self.time_now = rospy.get_rostime()

        rospy.loginfo("Initiating the release of fire-retardant liquid")
        
        rospy.loginfo("Focusing on peripheral area")

        for _ in range(1,6):
            self.rotateFormation(0, 0, 30)
            self.scaleFormation(0.9, 0.9, 1)
            self.applyFormation()

            while not self.is_on_formation():
                pass
    
        rospy.loginfo("Advancing to the center")

        for _ in range(1,3):
            self.rotateFormation(0, 0, 20)
            self.scaleFormation(0.75, 0.75, 1)
            self.applyFormation()

            while not self.is_on_formation():
                pass

        rospy.loginfo("Final Phase: Going closer to the center")

        for _ in range(1,1):
            self.rotateFormation(0, 0, 15)
            self.scaleFormation(0.6, 0.6, 1)
            self.applyFormation()
            
            while not self.is_on_formation():
                pass

        rospy.loginfo("Firefighting was a success")

        rospy.loginfo("Finishing the release of fire retardant liquid")

        rospy.sleep(2)


    def returnBase(self) -> None:

        rospy.loginfo("Iniciating new formation")

        centered_uav = self.uavs[0]
        another_uav = self.uavs[1]

        angle = helper.get_angle(centered_uav.pos_x, centered_uav.pos_y, another_uav.pos_x, another_uav.pos_y)

        self.setFormation(10, [centered_uav.pos_x, centered_uav.pos_y, another_uav.pos_z], start_angle=angle)
        self.applyFormation()
        
        while not self.is_on_formation():
            pass
        
        rospy.loginfo("Going to base")

        self.setFormation(10, [self.base_position_x, self.base_position_y, centered_uav.pos_z], start_angle=angle)
        self.applyFormation()

        while not self.is_on_formation():
            pass

        self.setFormation(5, [self.base_position_x, self.base_position_y, centered_uav.pos_z], start_angle=angle)
        self.applyFormation()

        while not self.is_on_formation():
            pass

        rospy.loginfo("Base reached")

        rospy.loginfo("Landing")
        self.land_all()
