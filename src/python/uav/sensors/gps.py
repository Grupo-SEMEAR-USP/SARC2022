#!/usr/bin/env python3

# Import Libraries
import numpy as np
import rospy

# Importing Ros Messages
from nav_msgs.msg import Odometry


class uavGPS:

    def __init__(self, node_name: str, subscriber_name: str):
        
        self.node_name = node_name
        self.subscriber_name = subscriber_name
        
        self.odometry_msg = None
        self.state = None
        self.position = None
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None
        self.heading = None

        self.sub = rospy.Subscriber(name = subscriber_name,
                                    data_class = Odometry,
                                    callback = self.read_odometry_msgs)
        
        #UAV Path/Trajectory
        self.current_path = None
        
        self.i_did_detect_fire: dict = {    "time": [],
                                            "fire_img": [],
                                            "fire_area": [],
                                            "x": [],
                                            "y": [],
                                            "z": []}

        self.previous_positions: dict = {   'time': [],
                                            'x': [],
                                            'y': [],
                                            'z': []}

        self.aux_vars_dict: dict = {'var1': np.pi}
    
    
    def read_odometry_msgs(self, odom_msg: Odometry) -> None:
        self.odometry_msg = odom_msg
        
        self.position = odom_msg.pose.pose.position
        self.pos_x = self.position.x
        self.pos_y = self.position.y
        self.pos_z = self.position.z
        self.heading = 0.0

        self.state = [self.pos_x, self.pos_y, self.pos_z, self.heading]

    def fire_detection_mapping( self,
                                fire_pixel_area: float,
                                min_area_threshold: float,
                                img_to_save: np.ndarray) -> bool:
        ''' 
            If an uav detected fire, save its state (the image, fire area, position, time, etc)
        '''
        did_i_detect_fire = fire_pixel_area and (fire_pixel_area > min_area_threshold) 

        if did_i_detect_fire:

            self.i_did_detect_fire['fire_img'].append(img_to_save)
            self.i_did_detect_fire['fire_area'].append(fire_pixel_area)
            self.i_did_detect_fire['x'].append(self.pos_x)
            self.i_did_detect_fire['y'].append(self.pos_y)
            self.i_did_detect_fire['z'].append(self.pos_z)
            self.i_did_detect_fire['time'].append(rospy.get_rostime())
        
        else:
            pass

        return did_i_detect_fire

    def save_xyz_position(self, t0: float, time_now: float, rate_hz: float = 0.5) -> None:
        
        '''

            Salva data em append_here a cada 1/rate_hz secs.
            Ex: salva self.position a cada 1s para mapear a trajetoria do drone

            Obs: rate_hz só pode ser tal que 1/rate_hz é inteiro diferente
            de zero. Logo, num_data/secs >= 1.

            #! vec.append() (built in) é muito mais rápido que np.append(array, obj)
        '''

        now_secs = time_now
        time_passed = now_secs - t0
        epoch = int(1/rate_hz)

        if ((time_passed % epoch == 0) and now_secs != self.aux_vars_dict['var1']):            
            
            self.previous_positions['x'].append(self.pos_x)
            self.previous_positions['y'].append(self.pos_y)
            self.previous_positions['z'].append(self.pos_z)
            self.previous_positions['time'].append(time_passed)

            self.aux_vars_dict['var1'] = now_secs

    def get_travel_points(self) -> dict: 
        return self.previous_positions

    def update_state(self) -> None:
        pass    
