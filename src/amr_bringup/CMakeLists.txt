cmake_minimum_required(VERSION 3.0.2)
project(amr_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  std_msgs
)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/odom_tf_bridge.py
  scripts/interface_watchdog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
