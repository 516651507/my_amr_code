cmake_minimum_required(VERSION 3.0.2)
project(amr_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  std_srvs
  std_msgs
)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/mapping_assist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/save_map.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
