cmake_minimum_required(VERSION 3.0.2)
project(amr_obstacle_fusion)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  costmap_2d
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES amr_fusion_layer
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2_ros tf2_sensor_msgs costmap_2d pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(amr_fusion_layer src/fusion_obstacle_layer.cpp)
add_dependencies(amr_fusion_layer ${catkin_EXPORTED_TARGETS})
target_link_libraries(amr_fusion_layer ${catkin_LIBRARIES})

add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
add_dependencies(sensor_fusion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_fusion_node ${catkin_LIBRARIES})

install(TARGETS amr_fusion_layer sensor_fusion_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES fusion_layer.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
