# 待你确认的硬件信息

全新项目里，下列项会直接影响 launch 能否一次跑通。知道答案后可在对应 yaml/launch 里改默认值，或告诉维护者合并进仓库。

## 底盘 / 串口

1. RK3588 上 STM32 实际设备是 **`/dev/ttyS4`** 还是 **`/dev/ttyUSB0`**？
2. 自动/手动切换：是否必须 **互斥**（只有 auto 模式才听 `/autoVel_cmd`）？当前 `amr_chassis` 用 `auto_cmd_timeout` 覆盖策略。
3. STM32 是否使用 **`MotorCommand.spin_speed`**？当前恒发 `0`。
4. 现场 **最大允许线速度** 是 1.5 m/s 还是固件仍限 0.25 m/s？

## 蓝海 LDS-50C-E

5. bluesea2 启动后原始扫描话题是 **`/scan`** 还是已在 **`/lidar1/laser_fan`**？
6. RK 接雷达的网口名是 **`eth1`** 吗？IP 是否固定 **192.168.158.15/24**？
7. 是否需要 **角度裁剪 / 强度过滤**（可在 `amr_lidar/config/lds50ce.yaml` 调）？

## 奥比中光大白 DW2 ×2

8. 你计划用的 ROS1 驱动是哪一个？
   - [OrbbecSDK_ROS1](https://github.com/orbbec/OrbbecSDK_ROS1)
   - `orbbec_camera` / `astra_camera` 其它分支
   - 厂商提供的 `camera_ws` 包名与 launch 文件名
9. 两路 USB 的 **device path / serial**（例如 `1-1.2` 与 `1-1.3`）？
10. 点云 `frame_id` 实际名称（用于 TF 与 `publish_description_tf`）？
11. 点云话题是否 **确定** 为 `/camera/depth/points_cam` 与 `/camera2/depth/points_cam`？

## 机械 / TF

12. **laser_link** 相对 `base_link` 的实测外参（xyz/rpy）？
13. 两台 DW2 相对 `base_link` 的实测外参？
14. 升降机构最大高度是否改变 **footprint**（导航多边形）？

---

回复时可按编号逐条答；涉及外参可给 CAD 或测量值。
