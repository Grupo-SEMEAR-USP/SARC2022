from uav import UAV

import numpy as np
import rospy
import cv2

from geometry_msgs.msg import Point

def create_patrol_route(initialPos: Point, altitude: float, mapSize: tuple, steps: int) -> list:
    
    path = []
    pos0 = [initialPos.x, initialPos.y, altitude, 0.0]
    radius = min(mapSize)

    path.append(pos0)

    if steps % 2:
        steps += 1

    for i, angle in enumerate(np.linspace(0.0, 2*np.pi, steps, False)):
        x = initialPos.x - np.sin(angle) * radius
        y = initialPos.y + np.cos(angle) * radius
        
        newPos = [x, y, altitude, 0.0]
        path.append(newPos)
        
        if i % 2: 
            path.append(pos0)
    
    return path
    

def main():
  
    myUav = UAV(node_name = 'uav1', uav_id= '1')

    while not myUav.position:
        myUav.update_state()

    trajectory = create_patrol_route(myUav.position, 30.0, (100, 100), 8)

    myUav.trajectory_generation(trajectory, 1)
    myUav.start_trajectory()

    while not rospy.is_shutdown():
        myUav.update_state()

        #print(f'Vi fogo: {myUav.did_i_detect_fire}')
        if myUav.did_i_detect_fire:
            cv2.imwrite('images/im.jpg', myUav.i_did_detect_fire['fire_img'][0])
            myUav.stop_trajectory()
            break


if __name__ == '__main__':
    main()
