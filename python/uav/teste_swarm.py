from swarm import Swarm

import rospy

def main():
    
    swarm = Swarm(9, 'swarm_node', spawn=True)

    if swarm.uavs:
        while not rospy.is_shutdown():
            swarm.update()

if __name__ == '__main__':
    main()