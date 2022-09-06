#!/usr/bin/env python3

# Importing Objects
from swarm_package.sensors.camera import uavCamera
from swarm_package.sensors.gps import uavGPS

# Importing Python Files
from swarm_package import helper

# Import Libraries
import numpy as np
import logging
import rospy

# Importing Ros Messages
from mrs_msgs.msg import Path, Reference, VelocityReferenceStamped, VelocityReference
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import Header

# Importing Ros Services
from mrs_msgs.srv import ReferenceStampedSrv, TrajectoryReferenceSrv, PathSrv, VelocityReferenceStampedSrv, String
from mrs_msgs.srv import ReferenceStampedSrvResponse, PathSrvResponse
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse

# Importing Gazebo Service to apply force 
from gazebo_msgs.srv import ApplyBodyWrench
#TODO: Verificar a possibilidade de pegar a massa do drone (f450) via script
#! A forca para anular o peso do drone deve ser contínua e nao em forma de impulsos!


class UAV:

    def __init__(self, uav_id: str, start_init: bool = True):
        
        self.uav_id = uav_id
        self.uav_name = f'uav{uav_id}'
        self.node_name = f'node_{self.uav_name}'

        self.camera = uavCamera(img_width = 640, img_height = 360,
                                node_name = 'uavs_imgs',
                                subscriber_name = f'/{self.uav_name}/rgbd_down/color/image_raw')


        self.gps = uavGPS(  node_name = 'odom_msgs',
                            subscriber_name = f'/{self.uav_name}/odometry/odom_gps')

        self.position = None
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None

        self.min_area_threshold = 10000
        
        self.previous_positions: dict = {   'time': [],
                                            'x': [],
                                            'y': [],
                                            'z': []}

        '''
            Save Img, area of fire detected, xyz position and the time

            'img': cv2.mat (np.array)
            'img_area': float
            i_did_detect_fire_here: list = [list of self.position]
            i_did_detect_fire_time: list = [list of rospy.get_rostime()]

        '''
        self.did_i_detect_fire: bool = False
        self.i_did_detect_fire: dict = {    "time": [],
                                            "fire_img": [],
                                            "fire_area": [],
                                            "x": [],
                                            "y": [],
                                            "z": []}
        


        if start_init:
            rospy.init_node(name = self.node_name)
            self.t0 = rospy.get_rostime()
            self.time_now = rospy.get_rostime()      

        self.goto_point_seq = 0
        self.waiting_time = None
        self.saving_data = False

        # Permite aplicar uma forca no drone
        self.wrench = Wrench()

        self.aux_vars_dict: dict = {'aux_var1': np.pi}

        #rospy.loginfo(f'UAV{self.uav_id} INIT SUCCEEDED')
        logging.debug(f'UAV{self.uav_id} INIT SUCCEEDED')

    def configure(self) -> None:
        
        logging.debug(f"Waiting MRS services UAV{self.uav_id} [...]")
        rospy.loginfo(f"Waiting MRS services UAV{self.uav_id} [...]")

        '''
            Tem que testar se essa é a estrutura correta de usar os serviços, 
            colocando todos na funcao configure().
            
        '''
        #https://answers.ros.org/question/11047/applying-a-force-to-a-rigid-body/
        rospy.wait_for_service(f'gazebo/apply_apply_body_wrench')
        self.force = rospy.ServiceProxy('gazebo/apply_body_wrench', ApplyBodyWrench)

        rospy.wait_for_service(f'{self.uav_name}/uav_manager/takeoff', timeout = None)

        rospy.wait_for_service(f'{self.uav_name}/uav_manager/land', timeout = None)
        self.land_here = rospy.ServiceProxy(f'{self.uav_name}/uav_manager/land', Trigger)

        rospy.wait_for_service(f'{self.uav_name}/uav_manager/land_there', timeout = None)
        self.land_there = rospy.ServiceProxy(f'{self.uav_name}/uav_manager/land_there', ReferenceStampedSrv)
        
        rospy.wait_for_service(f'{self.uav_name}/control_manager/reference', timeout = None)
        self.fly_to_xyz_in_a_given_frame = rospy.ServiceProxy(f'{self.uav_name}/control_manager/reference', ReferenceStampedSrv)

        rospy.wait_for_service(f'{self.uav_name}/control_manager/trajectory_reference', timeout = None)
        self.fly_along_trajectory = rospy.ServiceProxy(f'{self.uav_name}/control_manager/trajectory_reference', TrajectoryReferenceSrv)

        rospy.wait_for_service(f'{self.uav_name}/control_manager/velocity_reference', timeout = None)
        self.fly_with_velocity = rospy.ServiceProxy(f'{self.uav_name}/control_manager/velocity_reference', VelocityReferenceStampedSrv)

        rospy.wait_for_service(f'{self.uav_name}/control_manager/goto_trajectory_start', timeout = None)
        self.fly_to_1st_xyz_in_trajectory = rospy.ServiceProxy(f'{self.uav_name}/control_manager/goto_trajectory_start', Trigger)

        rospy.wait_for_service(f'{self.uav_name}/control_manager/start_trajectory_tracking', timeout = None)
        self.start_trajectory_tracking = rospy.ServiceProxy(f'{self.uav_name}/control_manager/start_trajectory_tracking', Trigger)

        rospy.wait_for_service(f'{self.uav_name}/control_manager/stop_trajectory_tracking', timeout = None)
        self.stop_trajectory_tracking = rospy.ServiceProxy(f'{self.uav_name}/control_manager/stop_trajectory_tracking', Trigger)

        rospy.wait_for_service(f'{self.uav_name}/control_manager/resume_trajectory_tracking', timeout = None)
        self.resume_trajectory_tracking = rospy.ServiceProxy(f'{self.uav_name}/control_manager/resume_trajectory_tracking', Trigger)

        rospy.wait_for_service(f'{self.uav_name}/trajectory_generation/path', timeout = None)
        self.path = rospy.ServiceProxy(f'{self.uav_name}/trajectory_generation/path', PathSrv)

        rospy.wait_for_service(f'{self.uav_name}/odometry/change_alt_estimator_type_string', timeout = None)
        self.change_estimator = rospy.ServiceProxy(f'{self.uav_name}/odometry/change_alt_estimator_type_string', String)

        res = self.change_estimator("BARO")

        while not res.success:
            res = self.change_estimator("BARO")

    
    def start_saving_data(self) -> None:
        self.saving_data = True

    def stop_saving_data(self) -> None:
        self.saving_data = False
    
    def update_state(self, t0: float, time_now: float) -> None:
        
        self.camera.update_state()
        self.gps.update_state()

        self.position = self.gps.position
        self.pos_x = self.gps.pos_x
        self.pos_y = self.gps.pos_y
        self.pos_z = self.gps.pos_z

        if self.uav_id == 1:
            self.camera.display_img(view_red=True)
        
        self.did_i_detect_fire = self.gps.fire_detection_mapping(   fire_pixel_area = self.camera.max_fire_area,
                                                                    min_area_threshold = self.min_area_threshold,
                                                                    img_to_save = self.camera.cv_img)

        if self.saving_data:
            self.gps.save_xyz_position(t0, time_now, rate_hz = 1.0)

    def apply_force(self, force_x, force_y, force_z, duration_sec,torque_x = 0, torque_y = 0, torque_z = 0) -> None:

            self.wrench.force.x = force_x
            self.wrench.force.y = force_y
            self.wrench.force.z = force_z
            self.wrench.torque_x = torque_x
            self.wrench.torque_y = torque_y
            self.wrench.torque_z = torque_z

            #call service
            #! Nao sei se tá certo o path do base_link
            self.force(body_name = f'{self.uav_name}/base_link', wrench = self.wrench, duration = rospy.Duration(secs = duration_sec))
        

    def get_travel_points(self) -> dict:
        return self.gps.get_travel_points()
    
    #TODO: Verificar a possibilidade de passar as funcoes de trajetoria 
    #para a classe uavGPS
    def trajectory_generation(self, points: list, id: int) -> PathSrvResponse:
         
        # Defining the services parameters
        
        msg_trajectory = Path()
        # Defining the Header
        msg_trajectory.header = Header()
        msg_trajectory.header.seq = 0
        msg_trajectory.header.stamp = rospy.get_rostime()
        msg_trajectory.header.frame_id = 'gps_origin'

        msg_trajectory.input_id = id #!Trocar o nome dessa entrada (id)

        msg_trajectory.use_heading = True
        msg_trajectory.fly_now = False
        msg_trajectory.stop_at_waypoints = False
        msg_trajectory.loop = False

        msg_trajectory.override_constraints = False

        msg_trajectory.relax_heading = False
        
        for point in points:
            ref = Reference()

            ref.position = Point()
            ref.position.x = point[0]
            ref.position.y = point[1]
            ref.position.z = point[2]

            ref.heading = point[3]

            msg_trajectory.points.append(ref)

        return self.path(msg_trajectory)

        
    def start_trajectory(self) -> TriggerResponse:

        self.fly_to_1st_xyz_in_trajectory()
        return self.start_trajectory_tracking()

    def stop_trajectory(self) -> TriggerResponse:

        return self.stop_trajectory_tracking()

    def go_to_point(self, position: list) -> ReferenceStampedSrvResponse:
        
        msg_point_header = Header()
        msg_point_reference = Reference()

        msg_point_header.seq = self.goto_point_seq
        msg_point_header.stamp = rospy.get_rostime()
        msg_point_header.frame_id = 'gps_origin'

        msg_point_reference.position = Point()
        msg_point_reference.position.x = position[0]
        msg_point_reference.position.y = position[1]
        msg_point_reference.position.z = position[2]
        msg_point_reference.heading = position [3]

        self.goto_point_seq += 1

        return self.fly_to_xyz_in_a_given_frame(msg_point_header, msg_point_reference)

    def fly_velocity(self, velocity: float, altitude: float) -> None:

        #velocity should be a float x,y,z vector
        
        msg_velocity = VelocityReferenceStamped()
        msg_velocity.header = Header()
        msg_velocity.reference = VelocityReference()

        msg_velocity.header.seq = 0
        msg_velocity.header.stamp = rospy.get_rostime()
        msg_velocity.header.frame_id = 'gps_origin'

        msg_velocity.reference.altitude = altitude
        msg_velocity.reference.heading = 0.0
        msg_velocity.reference.heading_rate = 0.0
        msg_velocity.reference.use_altitude = True
        msg_velocity.reference.use_heading = False
        msg_velocity.reference.use_heading_rate = False

        msg_velocity.reference.velocity = velocity
        
        res = self.fly_with_velocity(msg_velocity)

    def land_now(self) -> None:
        self.land_here()

    def land_position(self, position: list) -> None:

        msg_point_header = Header()
        msg_point_reference = Reference()

        msg_point_header.seq = 0
        msg_point_header.stamp = rospy.get_rostime()
        msg_point_header.frame_id = 'gps_origin'

        msg_point_reference.position = Point()
        msg_point_reference.position.x = position[0]
        msg_point_reference.position.y = position[1]
        msg_point_reference.heading = position [2]

        self.land_there(msg_point_header, msg_point_reference)

    def is_on_point(self, update: bool, t0: float, time_now: float, position: list) -> bool:
        if update == 1:
            self.update_state(t0, time_now)
        x = position[0]
        y = position[1]
        z = position[2]

        return helper.is_close_enough(self.pos_x, x) and helper.is_close_enough(self.pos_y, y) and helper.is_close_enough(self.pos_z, z)
     
    def is_on_point_for_time(self, t0: float, time_now: float, time: float,  position: list) -> bool:
        self.update_state(t0, time_now)
        x = position[0]
        y = position[1]
        z = position[2]

        if helper.is_close_enough(self.pos_x, x) and helper.is_close_enough(self.pos_y, y) and helper.is_close_enough(self.pos_z, z):
            if not self.waiting_time:
                self.waiting_time = time_now

            if time_now - self.waiting_time < time:
                return False
            else:
                self.waiting_time = None
                return True

