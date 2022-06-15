from uav import UAV  # importando do arquivo uav.py a classe uav
import rospy
from swarm import Swarm
def main():
  
  # v = 1
  # meu_uav1 = UAV(node_name = 'uav1', uav_id= '1')

  # trajectory_1 = [[0.0, 0.0, 30.0, 0.0], [0.0, 30.0, 30.0, 0.0], [30.0, 30.0, 30.0, 0.0], [30.0, 0.0, 30.0, 0.0]]

  # point_1 = [30.0, 30.0, 30.0, 0.0]

  # while not rospy.is_shutdown():
  #   meu_uav1.update_state()
  #   if v == 1:
  #     meu_uav1.go_to_point(point_1)
  #     v = 0

  # meu_uav1.trajectory_generation(trajectory_1, 1)
  # meu_uav1.start_trajectory()
  # meu_uav1.land_position([15.0, 17.0, 0.0])
  form = bool
  #swarm = Swarm(8,('uav1','uav2','uav3','uav4','uav5','uav6','uav7','uav8'), node_name = 'swarm')
  swarm = Swarm(8, 'swarm', 1, False)
  swarm.setFormation('circle', 8, 15)
  swarm.applyFormation()
  while form!='true':
    form = swarm.is_on_formation()
  rospy.loginfo("Uavs are on point")
  rospy.sleep(10)
  swarm.rotateFormation(0, 0, 30)
  swarm.scaleFormation(0.9, 0.9, 1)
  swarm.applyFormation()
  while form!='true':
        form = swarm.is_on_formation()
  rospy.sleep(3)
  swarm.rotateFormation(0, 0, 30)
  swarm.scaleFormation(0.9, 0.9, 1)
  swarm.applyFormation()
  while form!='true':
        form = swarm.is_on_formation()
  rospy.sleep(3)
  swarm.rotateFormation(0, 0, 30)
  swarm.scaleFormation(0.9, 0.9, 1)
  swarm.applyFormation()
  while form!='true':
        form = swarm.is_on_formation()
  rospy.sleep(3)
  swarm.rotateFormation(0, 0, 30)
  swarm.scaleFormation(0.9, 0.9, 1)
  swarm.applyFormation()
  while form!='true':
        form = swarm.is_on_formation()
  rospy.sleep(3)
  swarm.rotateFormation(0, 0, 30)
  swarm.scaleFormation(0.9, 0.9, 1)
  swarm.applyFormation()
  rospy.loginfo("Formation rotation was a sucess")
  
if __name__ == '__main__':
  main()
