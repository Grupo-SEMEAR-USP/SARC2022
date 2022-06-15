import rospy
import numpy as np
from nav_msgs.msg import Odometry

class uavGPS:

    def __init__(self, node_name: str, subscriber_name: str):
        
        self.node_name = node_name
        self.subscriber_name = subscriber_name
        
        self.odometry_msg = None
        self.position = None
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None

        # rospy.init_node(name = node_name, anonymous=True) 
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

        #store random data in 'aux_vars_dict' if needed
        self.aux_vars_dict: dict = {'var1': np.pi}
        
        self.t0 = rospy.get_rostime()
        self.time_now = rospy.get_rostime()
    
    
    def read_odometry_msgs(self, odom_msg: Odometry) -> None:
        self.odometry_msg = odom_msg
        
        self.position = odom_msg.pose.pose.position
        self.pos_x = self.position.x
        self.pos_y = self.position.y
        self.pos_z = self.position.z


    @staticmethod
    def circle_equation(angle_deg: np.array or float,
                        x_center: float,
                        y_center: float,
                        radius: float) -> np.array or float:
        '''
            x = R * cos(theta) + x0
            y = R * sin(theta) + y0
        '''

        angle_rad = np.pi * angle_deg/180.0
        
        #xOy reference (x0, y0) = (0, 0)
        x0_values = radius * np.cos(angle_rad)
        y0_values = radius * np.sin(angle_rad)

        #xO'y reference (x0', y0') = (x_center, y_center)
        #xOy -> xO'y
        x_values = x0_values + x_center
        y_values = y0_values + y_center

        return x_values, y_values


    
    def circular_trajectory_pts(self,
                                x_center: int or float,
                                y_center: int or float,
                                radius: int or float,
                                num_of_pts: int = 4) -> float or np.array:

        ''' 
            Equally spaced points of a circular trajectory, given a number of
            points.
        '''

        step = 360/num_of_pts
        angles_deg = np.array([i*step for i in range(num_of_pts)])
        
        xs_of_circ_trajectory, ys_of_circ_trajectory = self.circle_equation( angle_deg = angles_deg,
                                                                            x_center = x_center,
                                                                            y_center = y_center,
                                                                            radius = radius)
        return xs_of_circ_trajectory, ys_of_circ_trajectory


    def fire_detection_mapping( self,
                                fire_pixel_area: float,
                                min_area_threshold: float,
                                img_to_save: np.array) -> bool:
        ''' 
            If an uav detected fire, save its state (the image, fire area, position, time, etc)
        '''
        #Fire detection flag
        #!Certeza que retorna booleano?
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

        if ((time_passed % epoch == 0) and now_secs != self.aux_vars_dict['var1']):            
            
            self.previous_positions['x'].append(self.pos_x)
            self.previous_positions['y'].append(self.pos_y)
            self.previous_positions['z'].append(self.pos_z)
            self.previous_positions['time'].append(time_passed)

            self.aux_vars_dict['var1'] = now_secs

    def update_state(self) -> None:

        self.time_now = rospy.get_rostime()
    
    
