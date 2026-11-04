cmake_minimum_required(VERSION 3.0.2)
project(amr_web)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs sensor_msgs std_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/web_api_server.py
  scripts/web_routes.py
  scripts/config_store.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
