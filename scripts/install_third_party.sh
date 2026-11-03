#!/bin/bash
# 在 RK3588 / Ubuntu 20.04 + ROS Noetic 上拉取第三方驱动源码到工作空间 src/
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${SRC_DIR:-${ROOT}/src}"
mkdir -p "${SRC}"

clone_if_missing() {
  local dir="$1"
  local url="$2"
  if [[ -d "${SRC}/${dir}" ]]; then
    echo "已有 ${dir}，跳过 clone"
  else
    git clone --depth 1 "${url}" "${SRC}/${dir}"
  fi
}

echo "=== 蓝海 LDS-50C-E：bluesea2 (ROS1) ==="
clone_if_missing bluesea2 https://github.com/BlueSeaLidar/bluesea2.git
echo "复制 launch 示例："
echo "  cp ${ROOT}/src/amr_bringup/launch/lidar_bluesea_lds50ce.driver.launch.example \\"
echo "     ${ROOT}/src/amr_bringup/launch/lidar_bluesea_lds50ce.driver.launch"

echo ""
echo "=== 奥比 DW2：Orbbec ROS1（请按型号选分支/仓库）==="
echo "常见："
echo "  git clone https://github.com/orbbec/OrbbecSDK_ROS1.git ${SRC}/OrbbecSDK_ROS1"
echo "安装后改 amr_camera/launch/dw2_dual.launch 里的 include。"

echo ""
echo "=== Python 串口依赖 ==="
echo "  sudo apt install -y python3-serial"

echo ""
echo "完成后： cd ${ROOT} && catkin_make && source devel/setup.bash"
