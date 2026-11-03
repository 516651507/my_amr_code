#!/bin/bash
# 蓝海 LDS-50C-E 网口连通性检查（与 amr_bringup/config/machine.yaml 默认一致）
set -euo pipefail
LIDAR_IP="${1:-192.168.158.98}"
HOST_IP="${2:-192.168.158.15}"
IFACE="${3:-eth1}"

echo "=== LDS-50C-E 网络检查 ==="
echo "期望：雷达 ${LIDAR_IP}，RK ${HOST_IP}（接口 ${IFACE}）"

if command -v ip >/dev/null 2>&1; then
  ip -4 addr show dev "${IFACE}" 2>/dev/null || echo "警告: 未找到接口 ${IFACE}"
fi

if ping -c 2 -W 2 "${LIDAR_IP}" >/dev/null 2>&1; then
  echo "OK: ping ${LIDAR_IP} 成功"
else
  echo "FAIL: ping ${LIDAR_IP} 失败。请检查 eth 接线、雷达上电、RK 是否配 ${HOST_IP}/24" >&2
  exit 1
fi

echo "下一步（在已 source ROS 的机器上）："
echo "  rostopic hz /lidar1/scan_filtered"
echo "  roslaunch amr_bringup lidar_bluesea_lds50ce.launch   # 仅加载参数模板"
echo "  # 启动 bluesea2：复制 launch/lidar_bluesea_lds50ce.driver.launch.example 后 start_driver:=true"
