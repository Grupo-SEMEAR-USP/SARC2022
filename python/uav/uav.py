#!/usr/bin/env python3

# Importing Objects
from sensors.camera import uavCamera
from sensors.gps import uavGPS

# Importing Functions
import helper

# Import Libraries
import pandas as pd
import numpy as np
import logging
import rospy
import cv2
import os

# Importing Ros Messages
from mrs_msgs.msg import Path, Reference, VelocityReferenceStamped, VelocityReference
from geometry_msgs.msg import Point
from std_msgs.msg import Header

# Importing Ros Services
from mrs_msgs.srv import ReferenceStampedSrv, TrajectoryReferenceSrv, PathSrv, VelocityReferenceStampedSrv, String
from mrs_msgs.srv import ReferenceStampedSrvResponse, PathSrvResponse
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse



'''
    A classe UAV deve facilitar o o gerenciamento dos sistemas que compoe cada 
    drone (visao, gps, etc). Tambem deve permitir construir o swarm de forma 
    mais facil.

'''

#TODO:
'''
    #! Redistribuir as funcoes entre os arquivos uav.py e sensors.
    -> Funcoes relacionadas a posicao e trajetoria em uav.GPS
    -> funcoes relacionadas a imagem em uav.Camera

'''

class UAV:

    def __init__(self, uav_id: str, start_init: bool = True):
        
        self.uav_id = uav_id
        self.uav_name = f'uav{uav_id}'
        self.node_name = f'node_{self.uav_name}'

        self.camera = uavCamera(img_width = 720, img_height = 640,
                                node_name = 'uavs_imgs',
                                subscriber_name = f'/{self.uav_name}/rgbd_down/color/image_raw')


        self.gps = uavGPS(  node_name = 'odom_msgs',
                            subscriber_name = f'/{self.uav_name}/odometry/odom_gps')
        
        #TODO: add uma funcao que estabeleca os modos de voo
        self.current_flight_mode = None

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
        ##Add self.configure na main do script ao inves de coloca-lo e __init__
        # self.configure()

        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()

        self.goto_point_seq = 0

        #Store auxiliary variables in 'aux_vars_dict'
        self.aux_vars_dict: dict = {'aux_var1': np.pi}

        rospy.loginfo(f'UAV{self.uav_id} INIT SUCCEEDED')

    def configure(self):
        
        logging.debug("Waiting MRS services [...]")
        rospy.loginfo("Waiting MRS services [...]")

        ####* https://ctu-mrs.github.io/docs/system/uav_ros_interface.html

        #!Obs:
        '''
            Tem que testar se essa é a estrutura correta de usar os serviços, 
            colocando todos na funcao configure().
            
        '''
        
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

    
    def fire_detection_mapping( self,
                                fire_pixel_area: float,
                                min_area_threshold: float,
                                img_to_save: np.array):

        #! Essa funcao vai passar para a classe uavGPS                        
        ''' 
            If an uav detected fire, save its state (the image, fire area, position, time, etc)
        '''
        self.did_i_detect_fire = fire_pixel_area and (fire_pixel_area > min_area_threshold)

        if self.did_i_detect_fire:

            self.i_did_detect_fire['fire_img'].append(img_to_save)
            self.i_did_detect_fire['fire_area'].append(fire_pixel_area)
            self.i_did_detect_fire['x'].append(self.pos_x)
            self.i_did_detect_fire['y'].append(self.pos_y)
            self.i_did_detect_fire['z'].append(self.pos_z)
            self.i_did_detect_fire['time'].append(rospy.get_rostime())
        
        else:
            pass

    def save_xyz_position(self, rate_hz: float = 0.5) -> None:
        
        #! Essa funcao vai passar para a classe uavGPS
        '''

            Salva data em append_here a cada 1/rate_hz secs.
            Ex: salva self.position a cada 1s para mapear a trajetoria do drone

            Obs: rate_hz só pode ser tal que 1/rate_hz é inteiro diferente
            de zero. Logo, num_data/secs >= 1.

            #! vec.append() (built in) é muito mais rápido que np.append(array, obj)
        '''

        now_secs = self.time_now.secs
        time_passed = now_secs - self.t0.secs
        epoch = int(1/rate_hz)

        if ((time_passed % epoch == 0) and now_secs != self.aux_vars_dict['aux_var1']):            
            
            self.previous_positions['x'].append(self.pos_x)
            self.previous_positions['y'].append(self.pos_y)
            self.previous_positions['z'].append(self.pos_z)
            self.previous_positions['time'].append(time_passed)

            self.aux_vars_dict['aux_var1'] = now_secs

    
    def save_data(self, data: object, append_here: list, rate_hz: float = 0.5) -> None:
        
        '''

            Salva data em append_here a cada 1/rate_hz secs.
            Ex: salva self.position a cada 1s para mapear a trajetoria do drone

            Obs: rate_hz só pode ser tal que 1/rate_hz é inteiro diferente
            de zero. Logo, num_data/secs >= 1.

            #! vec.append() (built in) é muito mais rápido que np.append(array, obj)
        '''
        
        now_secs = self.time_now.secs
        time_passed = now_secs - self.t0.secs
        epoch = int(1/rate_hz)

        if ((time_passed % epoch == 0) and now_secs != self.aux_vars_dict['aux_var1']):
            
            append_here.append(data)
            self.aux_vars_dict['aux_var1'] = now_secs
    
    def update_state(self) -> None:
        
        self.time_now = rospy.get_rostime()
        
        self.camera.update_state()
        self.gps.update_state()

        self.position = self.gps.position
        self.pos_x = self.gps.pos_x
        self.pos_y = self.gps.pos_y
        self.pos_z = self.gps.pos_z

        if self.uav_id == 1:
            #self.camera.display_img(True)
            pass
        

        #TODO: add uma funcao na classe uavCamera que retorne a area do fogo 
        #detectado. Esta area é input da funcao fire_detection_tracker(
        self.did_i_detect_fire = self.gps.fire_detection_mapping(   fire_pixel_area = self.camera.max_fire_area,
                                                                    min_area_threshold = self.min_area_threshold,
                                                                    img_to_save = self.camera.cv_img)

        self.gps.save_xyz_position(rate_hz = 1.0)
       
                
        #rospy.loginfo("uav position:\n%s", self.position)

    
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

        #res = self.path(msg_trajectory)

        #print(f'Trajectory Generation\nSuccess: {res.success}\nMessage: {res.message}')

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

    def is_on_point(self, position: list) -> bool:
        self.update_state()
        x = position[0]
        y = position[1]
        z = position[2]

        return helper.is_close_enough(self.pos_x, x) and helper.is_close_enough(self.pos_y, y) and helper.is_close_enough(self.pos_z, z)
        
        
#uav de test
# meu_uav = UAV(node_name = 'meu_uav', uav_id = '01')

# if __name__ == '__main__':
    

    
#     while not rospy.is_shutdown():
#         meu_uav.update_state()
        

    #     meu_uav.camera.display_img()
    #     k = cv2.waitKey(1) & 0xff

    
    # cv2.destroyAllWindows()
        

