from uav import UAV  # importando do arquivo uav.py a classe uav
import rospy

def main():
  
  v = 1
  meu_uav1 = UAV(node_name = 'uav1', uav_id= '1')

  trajectory_1 = [[0.0, 0.0, 30.0, 0.0], [0.0, 30.0, 30.0, 0.0], [30.0, 30.0, 30.0, 0.0], [30.0, 0.0, 30.0, 0.0]]

  point_1 = [30.0, 30.0, 30.0, 0.0]

  # while not rospy.is_shutdown():
  #   meu_uav1.update_state()
  #   if v == 1:
  #     meu_uav1.go_to_point(point_1)
  #     v = 0

  meu_uav1.trajectory_generation(trajectory_1, 1)
  meu_uav1.start_trajectory()
  


if __name__ == '__main__':
  main()
