# catkin 工作空间顶层。机器人上已安装 Noetic 时，直接 catkin_make。
# 若 include 失败，改用官方符号链接：
#   ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)

if(NOT EXISTS "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake")
  message(FATAL_ERROR
    "未找到 ROS Noetic（/opt/ros/noetic）。本仓库是 ROS1 catkin 工作空间，"
    "请在 Ubuntu 20.04 + ros-noetic 上编译。说明见 README.md。")
endif()

set(CATKIN_TOPLEVEL TRUE)
include("/opt/ros/noetic/share/catkin/cmake/toplevel.cmake")
