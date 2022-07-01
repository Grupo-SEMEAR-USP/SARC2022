#!/usr/bin/python3

# Importing Objects
from swarm_package.swarm import Swarm

# Import Libraries
import rospy


def main():
    
    swarm = Swarm(9, 'swarm_node', spawn=True)

    if swarm.uavs:
        while not rospy.is_shutdown():
            swarm.update()
        
        if not swarm.points_saved:
            if input('Save?').lower() == 's':
                swarm.save_drones_travel_position()

if __name__ == '__main__':
    main()