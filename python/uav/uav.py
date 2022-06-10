#!/usr/bin/env python3
from turtle import heading
import numpy as np
import os
import cv2
#import pandas as pd
import logging
import rospy
from sensors.camera import uavCamera
from sensors.gps import uavGPS
# from mavros_msgs import srv

from geometry_msgs.msg import Point
#Importing mrs_services 
from mrs_msgs.srv import    ReferenceStampedSrv,    \
                            TrajectoryReferenceSrv,        \
                            PathSrv

#from mrs_msgs.msg import _ReferenceStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Header
from mrs_msgs.msg import Path, Reference



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

    def __init__(self, node_name: str, uav_id: str):
        
        self.uav_name = os.environ.get('UAV_NAME')
        self.node_name = node_name
        self.uav_id = uav_id

        self.camera = uavCamera(img_width = 720, img_height = 640,
                                node_name = 'uavs_imgs',
                                subscriber_name = '/uav1/rgbd_down/color/image_raw')


        self.gps = uavGPS(  node_name = 'odom_msgs',
                            subscriber_name = '/uav1/odometry/odom_gps')
        
        
        self.position = None
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None
        
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
        


        rospy.init_node(name = node_name)
        self.configure()

        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()

        #Store auxiliary variables in 'aux_vars_dict'
        self.aux_vars_dict: dict = {'aux_var1': np.pi}

        rospy.loginfo(f'UAV{self.node_name} INIT SUCCEEDED')

    def configure(self):
        
        logging.debug("Waiting MRS services [...]")
        rospy.loginfo("Waiting MRS services [...]")

        ####* https://ctu-mrs.github.io/docs/system/uav_ros_interface.html

        #!Obs:
        '''
            Tem que testar se essa é a estrutura correta de usar os serviços, 
            colocando todos na funcao configure().
            
        '''
        rospy.wait_for_service(f'{self.node_name}/uav_manager/takeoff', timeout = None)
        rospy.wait_for_service(f'{self.node_name}/uav_manager/land', timeout = None)
        
        rospy.wait_for_service(f'{self.node_name}/control_manager/reference', timeout = None)
        rospy.wait_for_service(f'{self.node_name}/control_manager/trajectory_reference', timeout = None)

        rospy.wait_for_service(f'{self.node_name}/control_manager/goto_trajectory_start', timeout = None)
        rospy.wait_for_service(f'{self.node_name}/control_manager/start_trajectory_tracking', timeout = None)
        rospy.wait_for_service(f'{self.node_name}/control_manager/stop_trajectory_tracking', timeout = None)
        rospy.wait_for_service(f'{self.node_name}/control_manager/resume_trajectory_tracking', timeout = None)

        rospy.wait_for_service(f'{self.node_name}/trajectory_generation/path', timeout = None)

        

        # try:
        # self.takeoff(f'{self.node_name}/uav_manager/takeoff', _Trigger)
        # self.land(f'{self.node_name}/uav_manager/land', _Trigger)
        # self.land_there(f'{self.node_name}/uav_manager/land_there', _ReferenceStamped)
        
        self.fly_to_xyz_in_a_given_frame = rospy.ServiceProxy(f'{self.node_name}/control_manager/reference', ReferenceStampedSrv)
        self.fly_along_trajectory = rospy.ServiceProxy(f'{self.node_name}/control_manager/trajectory_reference', TrajectoryReferenceSrv)

        self.fly_to_1st_xyz_in_trajectory = rospy.ServiceProxy(f'{self.node_name}/control_manager/goto_trajectory_start', Trigger)
        self.start_trajectory_tracking = rospy.ServiceProxy(f'{self.node_name}/control_manager/start_trajectory_tracking', Trigger)
        self.stop_trajectory_tracking = rospy.ServiceProxy(f'{self.node_name}/control_manager/stop_trajectory_tracking', Trigger)
        self.resume_trajectory_tracking = rospy.ServiceProxy(f'{self.node_name}/control_manager/resume_trajectory_tracking', Trigger)
        self.path = rospy.ServiceProxy(f'{self.node_name}/trajectory_generation/path', PathSrv)
        
        # except rospy.ServiceException as e:
        #     print('Service call failed: %s', e)

        
    
    def fire_detection_mapping( self,
                                fire_pixel_area: float,
                                min_area_threshold: float,
                                img_to_save: np.array):
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

        self.position = self.gps.odometry_msg.pose.pose.position
        self.pos_x = self.position.x
        self.pos_y = self.position.y
        self.pos_z = self.position.z

        self.camera.display_img()
        

        #TODO: add uma funcao na classe uavCamera que retorne a area do fogo 
        #detectado. Esta area é input da funcao fire_detection_tracker(
        self.fire_detection_mapping(fire_pixel_area = self.camera.max_fire_area,
                                    min_area_threshold = 0,
                                    img_to_save = self.camera.cv_img)

        self.save_xyz_position(rate_hz = 1.0)
       
                
        #rospy.loginfo("uav position:\n%s", self.position)
        
    def trajectory_generation(self, points: list, id: int) -> None:
         
        # Defining the services parameters
        
        msg_trajectory = Path()
        # Defining the Header
        msg_trajectory.header = Header()
        msg_trajectory.header.seq = 0
        msg_trajectory.header.stamp = rospy.get_rostime()
        msg_trajectory.header.frame_id = 'gps_origin'

        msg_trajectory.input_id = id

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

        res = self.path(msg_trajectory)

        print(f'Trajectory Generation\nSuccess: {res.success}\nMessage: {res.message}')

        
    def start_trajectory(self) -> None:

        self.fly_to_1st_xyz_in_trajectory()
        self.start_trajectory_tracking()

    def stop_trajectory(self) -> None:

        self.stop_trajectory_tracking()

    def go_to_point(self, position) -> None:
        
        msg_point_header = Header()
        msg_point_reference = Reference()

        msg_point_header.seq = 0
        msg_point_header.stamp = rospy.get_rostime()
        msg_point_header.frame_id = 'gps_origin'

        msg_point_reference.position = Point()
        msg_point_reference.position.x = position[0]
        msg_point_reference.position.y = position[1]
        msg_point_reference.position.z = position[2]
        msg_point_reference.heading = position [3]

        self.fly_to_xyz_in_a_given_frame(msg_point_header, msg_point_reference)

#uav de test
# meu_uav = UAV(node_name = 'meu_uav', uav_id = '01')

# if __name__ == '__main__':
    

    
#     while not rospy.is_shutdown():
#         meu_uav.update_state()
        

    #     meu_uav.camera.display_img()
    #     k = cv2.waitKey(1) & 0xff

    
    # cv2.destroyAllWindows()
        

