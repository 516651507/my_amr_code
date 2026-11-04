# 待确认（剩余项）

已确认：**/dev/ttyS4**、**eth1 + 192.168.158.15**、空载 **1.5 m/s** / 负载 **0.8 m/s**。

## 外参（标定）

见 **[extrinsics_zh.md](extrinsics_zh.md)**。请尽量提供：

1. 雷达中心相对车体中心：**前/后、左/右、高（米）** + 安装角  
2. 两台 DW2 各自相对车体中心的 **xyz + 俯仰**  
3. 若升降会影响占地，**最大 footprint**

## DW2 USB

两路 USB 在 RK3588 上的 **端口路径**（`lsusb -t` 或 udev），填入 `amr_camera/config/dw2_dual.yaml` 的 `usb_port`，否则双机可能抢同一设备。

## 自动/手动

Web/APP 是否需 **硬件 AUTO 模式** 才允许 `/autoVel_cmd`？当前为软件优先（auto 在 timeout 内覆盖 cmd_vel）。
