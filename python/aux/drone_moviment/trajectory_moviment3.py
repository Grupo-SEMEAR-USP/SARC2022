import numpy as np
import rospy
import os

from mrs_msgs.msg import Path, Reference
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from std_srvs.srv import Trigger
from mrs_msgs.srv import PathSrv

class UAVTest:
  def __init__(self, uav_name: str) -> None:

    self.uav_name = uav_name

    if self.uav_name == 'uav1':
      self.points = [[5.0, 0.0, 5.0, 0.0], [-5.0, 0.0, 5.0, 0.0]]
    else:
      self.points = [[-5.0, 0.0, 5.0, 0.0], [5.0, 0.0, 5.0, 0.0]]

    rospy.init_node('uav_moviment', anonymous=True)

    self.configure()

  def configure(self) -> None:
    
    rospy.loginfo("Waiting MRS services [...]")

    rospy.wait_for_service(f'{self.uav_name}/trajectory_generation/path', timeout=None)
    self.planner = rospy.ServiceProxy(f'{self.uav_name}/trajectory_generation/path', PathSrv)

    rospy.wait_for_service(f'{self.uav_name}/control_manager/start_trajectory_tracking', timeout=None)
    self.start_mov = rospy.ServiceProxy(f'{self.uav_name}/control_manager/start_trajectory_tracking', Trigger)

  def goto_start_point_v2(self) -> None:
    msg = Path()

    msg.header = Header()
    msg.header.seq = 0
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = ''

    msg.input_id = 1

    msg.use_heading = True
    msg.fly_now = True
    msg.stop_at_waypoints = False
    msg.loop = False

    msg.override_constraints = False

    msg.relax_heading = False

    msg.points = []
    point = self.points[0]

    ref = Reference()

    ref.position = Point()
    ref.position.x = point[0]
    ref.position.y = point[1]
    ref.position.z = point[2]

    ref.heading = point[3]

    msg.points.append(ref)

    #rospy.loginfo(msg)

    res = self.planner(msg)

    print(f'Goto Start Position -> Success: {res.success} - Message: {res.message}')

  def create_trajectory(self) -> None:
    msg = Path()

    msg.header = Header()
    msg.header.seq = 0
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = ''

    msg.input_id = 1

    msg.use_heading = True
    msg.fly_now = False
    msg.stop_at_waypoints = False
    msg.loop = False

    msg.override_constraints = False

    msg.relax_heading = False

    msg.points = []
    for point in self.points:
      ref = Reference()

      ref.position = Point()
      ref.position.x = point[0]
      ref.position.y = point[1]
      ref.position.z = point[2]

      ref.heading = point[3]

      msg.points.append(ref)

    #rospy.loginfo(msg)

    res = self.planner(msg)

    print(f'Trajectory Generation -> Success: {res.success} - Message: {res.message}')

  def start_trajectory(self) -> None:
    res = self.start_mov()

    print(f'Trajectory Start -> Success: {res.success} - Message: {res.message}')


def main():

  uav1 = UAVTest('uav1')
  uav2 = UAVTest('uav2')

  input('a')
  uav1.goto_start_point_v2()
  uav2.goto_start_point_v2()
  input('b')
  uav1.create_trajectory()
  uav2.create_trajectory()
  input('c')
  uav1.start_trajectory()
  uav2.start_trajectory()



if __name__ == '__main__':
  main()