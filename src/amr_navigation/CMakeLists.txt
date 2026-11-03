cmake_minimum_required(VERSION 3.0.2)
project(amr_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_smoother.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
