cmake_minimum_required(VERSION 3.2.0)
project(no_fire_squad)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  mrs_msgs
)

catkin_python_setup()

catkin_package()

install(
  DIRECTORY launch media models world_models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/sarc_map.py src/swarm_package/run_swarm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
