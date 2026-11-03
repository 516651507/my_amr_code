#!/bin/bash
# 保存当前 /map。优先走 mapping_assist 服务，没有该节点时回退到 map_saver。
set -euo pipefail
NAME="${1:-amr_map}"
if ! [[ "${NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "地图名只允许字母、数字、下划线和中划线: ${NAME}" >&2
  exit 1
fi

if rosservice list 2>/dev/null | grep -qx "/mapping_assist/save_map"; then
  rosparam set /mapping_assist/map_name "${NAME}"
  rosservice call /mapping_assist/save_map "{}"
  exit 0
fi

MAP_DIR="$(rospack find amr_mapping)/maps"
mkdir -p "${MAP_DIR}"
echo "mapping_assist 不在运行，改用 map_saver -> ${MAP_DIR}/${NAME}"
rosrun map_server map_saver -f "${MAP_DIR}/${NAME}" map:=/map
