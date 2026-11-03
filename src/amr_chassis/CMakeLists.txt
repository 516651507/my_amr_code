cmake_minimum_required(VERSION 3.0.2)
project(amr_chassis)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/chassis_serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
