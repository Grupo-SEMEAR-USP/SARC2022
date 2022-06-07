from mavros_msgs.srv import CommandTOL
from mrs_msgs.srv import Vec4
import rospy
import os

def main():
  rospy.init_node('uav_moviment', anonymous=True)

  uav_name = os.environ.get('UAV_NAME')
  
  rospy.loginfo("Waiting MRS services [...]")

  rospy.wait_for_service(f'{uav_name}/mavros/cmd/takeoff', timeout = None)

  #input('a')

  rospy.wait_for_service(f'{uav_name}/control_manager/goto', timeout = None)
  goto = rospy.ServiceProxy(f'{uav_name}/control_manager/goto', Vec4)

  #res = goto(goal = np.array([5.0, -15.0, 10.0, 0.0], np.float64))
  res = goto(goal = [0.0, 0.0, 15.0, 0.0])

  print("{} => {}".format(res.success, res.message))


if __name__ == '__main__':
  main()
