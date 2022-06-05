from uav import UAV  # importando do arquivo uav.py a classe uav
import rospy

def main():
  
  meu_uav1 = UAV(node_name = 'meu_uav1', uav_id= '1')
  meu_uav1.trajectory_generation()


  pass


if __name__ == '__main__':
  main()
