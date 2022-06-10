import rospy
import numpy as np
from nav_msgs.msg import Odometry

class uavGPS:

    def __init__(self, node_name: str, subscriber_name: str):
        
        self.node_name = node_name
        self.subscriber_name = subscriber_name
        
        self.position = 0 
        self.odometry_msg = None
        # rospy.init_node(name = node_name, anonymous=True) 
        self.sub = rospy.Subscriber(name = subscriber_name,
                                    data_class = Odometry,
                                    callback = self.callback)
        
        #store random data in 'data_container' if needed
        self.data_container: dict = {}

    def callback(self, odom_msg: Odometry) -> None:
        self.odometry_msg = odom_msg
            


    def update_state(self) -> None:
        pass
    
    
