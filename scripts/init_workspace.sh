#!/bin/bash
# 在已 source ROS Noetic 的终端里执行，检查工作空间能否被 catkin 识别。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "未找到 /opt/ros/noetic/setup.bash。请先安装 ROS Noetic。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [[ ! -f "${ROOT}/src/CMakeLists.txt" ]]; then
  ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "${ROOT}/src/CMakeLists.txt"
fi
echo "工作空间: ${ROOT}"
echo "下一步: cd ${ROOT} && catkin_make -DCMAKE_BUILD_TYPE=Release && source devel/setup.bash"
