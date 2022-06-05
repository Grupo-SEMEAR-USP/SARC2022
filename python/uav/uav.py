#!/usr/bin/env python3
import numpy as np
import cv2
import pandas as pd
import logging
import rospy
from sensors.camera import uavCamera
from sensors.gps import uavGPS


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
        
        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()

        #Store auxiliary variables in 'aux_vars_dict'
        self.aux_vars_dict: dict = {'aux_var1': np.pi}

    def configure(self):
        
        logging.debug("Waiting MRS services [...]")
        rospy.loginfo("Waiting MRS services [...]")

        #TODO: Listar os servicos utilizados
        #Takeoff
        rospy.wait_for_service(f'{self.node_name}/mavros/cmd/takeoff', timeout = 1)
        # self.takeoff(f'{self.node_name}/mavros/cmd/takeoff', srv.foo)
        

    def fire_detection_mapping( self,
                                fire_pixel_area: float,
                                min_area_threshold: float,
                                img_to_save: np.array):
        ''' 
            Track
        '''
        self.did_i_detect_fire = (fire_pixel_area > min_area_threshold)

        if self.did_i_detect_fire is True:

            self.i_did_detect_fire['fire_img'].append(img_to_save)
            self.i_did_detect_fire['fire_area'].append(fire_pixel_area)
            self.i_did_detect_fire['position'].append(self.position)
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
        

        #TODO: add uma funcao na classe uavCamera que retorne a area do fogo 
        #detectado. Esta area é input da funcao fire_detection_tracker()
        self.fire_detection_mapping(fire_pixel_area = -1,
                                    min_area_threshold = 0,
                                    img_to_save = None)

        self.save_xyz_position(rate_hz = 1.0)
       
                
        rospy.loginfo("uav position:\n%s", self.position)
        
        
        

# #uav de test
# meu_uav = UAV(node_name = 'meu_uav', uav_id = '01')

# if __name__ == '__main__':
    

    
#     while not rospy.is_shutdown():
#         meu_uav.update_state()
        

#     #     meu_uav.camera.display_img()
#     #     k = cv2.waitKey(1) & 0xff

#     #     if int(meu_uav.time_now.secs - meu_uav.t0.secs) == 120:
            
#     #         #save data
#     #         df_dict = { 'time': meu_uav.previous_positions['time'],
#     #                     'x': meu_uav.previous_positions['x'],
#     #                     'y': meu_uav.previous_positions['y'],
#     #                     'z': meu_uav.previous_positions['z']}
            
#     #         df = pd.DataFrame(df_dict)
#     #         df.to_csv('Positions.csv')

        
#     # cv2.destroyAllWindows()
        