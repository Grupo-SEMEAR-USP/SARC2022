import numpy as np
import rospy
import os

from mrs_msgs.msg import Path, Reference
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from std_srvs.srv import Trigger
from mrs_msgs.srv import PathSrv

class UAVTest:
  def __init__(self) -> None:
    self.points_1 = [[0.0, 0.0, 30.0, 0.0], [0.0, 30.0, 30.0, 0.0], [30.0, 30.0, 30.0, 0.0], [30.0, 0.0, 30.0, 0.0]]
    self.points_2 = [[(5*np.cos(t))/(1+np.sin(t)**2), (5*np.sin(t)*np.cos(t))/(1+np.sin(t)**2), 5.0, 0.0] for t in list(np.linspace(0, 2*np.pi, num=20))]

    self.uav_name = os.environ.get('UAV_NAME')

    rospy.init_node('uav_moviment', anonymous=True)

    self.configure()

  def configure(self) -> None:
    
    rospy.loginfo("Waiting MRS services [...]")

    rospy.wait_for_service(f'{self.uav_name}/trajectory_generation/path', timeout=None)
    self.planner = rospy.ServiceProxy(f'{self.uav_name}/trajectory_generation/path', PathSrv)

    rospy.wait_for_service(f'{self.uav_name}/control_manager/goto_trajectory_start', timeout=None)
    self.goto_start = rospy.ServiceProxy(f'{self.uav_name}/control_manager/goto_trajectory_start', Trigger)

    rospy.wait_for_service(f'{self.uav_name}/control_manager/start_trajectory_tracking', timeout=None)
    self.start_mov = rospy.ServiceProxy(f'{self.uav_name}/control_manager/start_trajectory_tracking', Trigger)

  def goto_start_point_v1(self) -> None:
    res = self.goto_start()

    print(f'Goto Start Position\nSuccess: {res.success}\nMessage: {res.message}')

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
    point = self.points_2[0]

    ref = Reference()

    ref.position = Point()
    ref.position.x = point[0]
    ref.position.y = point[1]
    ref.position.z = point[2]

    ref.heading = point[3]

    msg.points.append(ref)

    #rospy.loginfo(msg)

    res = self.planner(msg)

    print(f'Goto Start Position\nSuccess: {res.success}\nMessage: {res.message}')

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
    for point in self.points_2:
      ref = Reference()

      ref.position = Point()
      ref.position.x = point[0]
      ref.position.y = point[1]
      ref.position.z = point[2]

      ref.heading = point[3]

      msg.points.append(ref)

    #rospy.loginfo(msg)

    res = self.planner(msg)

    print(f'Trajectory Generation\nSuccess: {res.success}\nMessage: {res.message}')

  def start_trajectory(self) -> None:
    res = self.start_mov()

    print(f'Trajectory Start\nSuccess: {res.success}\nMessage: {res.message}')


def main():

  uav = UAVTest()

  input('teste1\n')

  input('passo 1')
  uav.goto_start_point_v2()
  input('passo 2')
  uav.create_trajectory()
  input('passo 3')
  uav.start_trajectory()

  input('\nteste2\n')

  input('passo 1')
  uav.create_trajectory()
  input('passo 2')
  uav.goto_start_point_v1()
  input('passo 3')
  uav.start_trajectory()


if __name__ == '__main__':
  main()