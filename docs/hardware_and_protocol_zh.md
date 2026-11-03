# 硬件与下层协议对接说明

本文档把 **STM32 ↔ RK3588 串口协议**、**蓝海 LDS-50C-E**、**奥比中光大白 DW2（双机）** 与本仓库 **amr_* 应用层** 的边界说清楚。驱动与 `mars_serial_server` 仍在原工作空间；这里只约定 ROS 话题、参数和启动顺序。

---

## 1. 分层关系

```
┌─────────────────────────────────────────────────────────────┐
│  amr_mapping / amr_navigation / amr_obstacle_fusion       │  ← 本仓库（应用层）
└───────────────────────────┬─────────────────────────────────┘
                            │ 只订阅/发布标准 ROS 话题
┌───────────────────────────▼─────────────────────────────────┐
│  mars_serial_server │ bluesea2 │ camera_ws (Orbbec DW2)     │  ← 现场已有 / 其它 catkin_ws
│  mars_tf_bridge     │ scan 过滤节点 → /lidar1/scan_filtered │
└───────────────────────────┬─────────────────────────────────┘
                            │ UART / UDP / USB
┌───────────────────────────▼─────────────────────────────────┐
│  STM32 底盘 │ LDS-50C-E │ DW2 ×2                            │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. STM32 ↔ RK3588 串口协议（与 mars_serial_server 对齐）

依据下层协议文档整理；**字节序小端**，帧格式：

`5A A5 | msgtype msgid length | payload | XOR校验 | 5C C5`

### 2.1 与导航相关的 msgtype / msgid

| 方向 | msgtype | msgid | 含义 | Payload |
|------|---------|-------|------|---------|
| STM32→RK | 2 (`CM_DEV_STA_TYPE`) | 1 | IMU | `IMUEncoder` 24B，当前 RK 主要用 `omg_z` |
| STM32→RK | 2 | 2 | 电池 | `BatteryDatas` 5×float32 |
| STM32→RK | 2 | 3 | 轮速编码器 | `MotorEncoder` 左右 int32 |
| RK→STM32 | 1 (`CM_DEV_CTL_TYPE`) | 2 | 电机 | `MotorCommand` 左右+spin double |

### 2.2 差速运动学（与 mars 代码一致）

**上报（编码器 → 线速度）：**

- 右轮线速度用 **left_speeder 取负**，左轮用 **right_speeder**（左右字段交叉，以现场代码为准）。
- `speed_rpm = encoder / 1000`，`v = speed_rpm * π * wheel_diameter / 60`（m/s）。

**下发（/cmd_vel、/autoVel_cmd → 电机）：**

```text
right_v = linear.x - angular.z * wheel_spacing / 2
left_v  = linear.x + angular.z * wheel_spacing / 2
motor.left  =  left_v  * 60 / (π * wheel_diameter)
motor.right = -right_v * 60 / (π * wheel_diameter)   // 右轮目标取负
```

本仓库 **不实现串口**，但 `amr_bringup/config/machine.yaml` 中的 **轮径 0.165 m、轮距 0.42 m** 必须与 `mars_serial_server` / `machine_params` 一致，否则 TEB、gmapping 噪声参数与真实运动不匹配。

### 2.3 串口设备

| 来源 | 设备 | 说明 |
|------|------|------|
| 硬件 brief | `/dev/ttyS4` @ 115200 | 彭飞板载 UART 接 STM32 |
| 协议文档默认 | `/dev/ttyUSB0` | 部分 mars 工程 USB 转串口 |

以 **现场 mars_serial_server 实际配置** 为准；本仓库在 `machine.yaml` 里默认写 `ttyS4`，并注释 USB 备选。

### 2.4 速度话题分工（必须遵守）

| 话题 | 谁发布 | 谁消费 |
|------|--------|--------|
| `/cmd_vel` | 遥控 / 手动节点 | mars_serial → STM32 |
| `/autoVel_cmd` | **本仓库** `cmd_smoother`（导航自动） | mars_serial → STM32 |
| `/wheel_odom` | mars（由 MotorEncoder 积分） | gmapping、amcl、TEB、`odom_tf_bridge` |

**不要**让导航栈发布 `/cmd_vel`，避免和手动抢控制权。

### 2.5 IMU / 电池（可选，未进本栈融合）

协议里 IMU、电池已由 STM32 上报；若 mars 已发布 ROS 话题，可在 `topics.yaml` 的 `optional_topics` 里加入看门狗监控。当前 **建图与 move_base 不依赖 IMU**；以后若要 `robot_localization` 融合，再单独开任务。

---

## 3. 蓝海雷达 LDS-50C-E（UDP + bluesea2）

### 3.1 型号与连接

- 型号：**LDS-50C-E**（蓝海 BlueSea，**以太网 UDP**，不是 YDLIDAR 串口驱动）。
- 驱动包：**ROS1 `bluesea2`**（[BlueSeaLidar/bluesea2](https://github.com/BlueSeaLidar/bluesea2)），launch 用 **`udp_lidar.launch`**。
- 现场网络（与 brief 一致）：

| 项 | 值 |
|----|-----|
| 雷达 IP | 192.168.158.98 |
| 雷达上报目标（RK 网口） | 192.168.158.15 |
| UDP 端口 | **6668**（常见配置；以 bluesea2 launch / 上位机为准） |
| RK 网口 | **eth1** 配 192.168.158.15/24 |

上电后：`ping 192.168.158.98`。

### 3.2 话题与坐标系（现场栈）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lidar1/laser_fan` | 驱动原始/扇区数据 | bluesea2 或中间节点 |
| `/lidar1/scan_filtered` | `sensor_msgs/LaserScan` | **本仓库建图、amcl、融合的唯一雷达输入** |
| `laser_link` | TF | 与 `mars_tf_bridge` 一致时多为 **base_link 单位变换** |

本仓库提供 **`amr_bringup/config/bluesea_lds50ce.yaml`** 作为 bluesea2 参数模板，以及 **`launch/lidar_bluesea_lds50ce.launch`**（可选启动驱动，需已安装 `bluesea2` 包）。

### 3.3 与 gmapping / 融合的距离参数

- LDS-50C-E 标称测距可达约 **50 m**；室内/仓库建议 **有效使用 25–30 m**（已在 `gmapping.yaml`、`fusion.yaml` 裁剪）。
- 角分辨率由驱动 `hard_resample_factor` / 软采样决定；融合节点默认 **0.5°** 重分桶，与驱动角步长不一致时仍安全，只是略粗。

---

## 4. 奥比中光大白 DW2 ×2（camera_ws）

### 4.1 型号与部署

- 型号：**Orbbec Dabai DW2**（深度相机 ×2）。
- 驱动通常在 **`~/camera_ws`** 单独 catkin 工作空间，与本仓库 **并行 source**：

```bash
source /opt/ros/noetic/setup.bash
source ~/camera_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash   # 或本 repo 的 devel
```

### 4.2 话题（与 brief 一致）

| 相机 | 点云话题 | 本栈用途 |
|------|----------|----------|
| 1 | `/camera/depth/points_cam` | `sensor_fusion_node` |
| 2 | `/camera2/depth/points_cam` | `sensor_fusion_node` |

**不**把原始深度图写进 gmapping 地图。

### 4.3 坐标系

- 点云 `header.frame_id` 必须能 TF 到 **`base_link`**（常见为 `camera_depth_optical_frame` / 驱动自定义名）。
- 若 DW2 驱动或 `mars_tf_bridge` **已在发** 相机 TF，启动导航/建图时设：

```bash
publish_description_tf:=false
```

- `amr_description` 里 DW2 的 link 名为占位，外参需 **标定后** 写入 xacro 或独立 static_transform_publisher。

### 4.4 融合高度窗

`fusion.yaml` 默认 **0.08–1.50 m**（base_link 高度）用于去掉地面与天花板；DW2 安装较低时可把 `min_height` 略增（如 0.10–0.12），减少地面噪点。

---

## 5. 推荐启动顺序（RK3588）

```bash
# 1) 底盘 + 里程计（mars，串口 ttyS4 或 USB）
roslaunch mars_bringup ...   # 以现场包名为准

# 2) TF 桥（若未合在 mars 里）
# mars_tf_bridge: base_link -> laser_link 等

# 3) 蓝海雷达（可选：本仓库模板）
roslaunch amr_bringup lidar_bluesea_lds50ce.launch start_driver:=true

# 4) 相机（camera_ws）
roslaunch ... # DW2 双机 launch

# 5) 应用层
roslaunch amr_mapping mapping.launch publish_description_tf:=false publish_wheel_odom_tf:=false
# 或
roslaunch amr_navigation navigation.launch map_file:=... publish_description_tf:=false
```

若 mars 已发布 `odom->base_link`，全程 **`publish_wheel_odom_tf:=false`**。

---

## 6. 配置文件索引（本仓库）

| 文件 | 内容 |
|------|------|
| `amr_bringup/config/machine.yaml` | 轮径、轮距、串口、速度上限、与协议一致的差速说明 |
| `amr_bringup/config/topics.yaml` | ROS 话题契约 + 看门狗 |
| `amr_bringup/config/bluesea_lds50ce.yaml` | LDS-50C-E / bluesea2 UDP 模板 |
| `amr_bringup/config/orbbec_dw2.yaml` | 双 DW2 话题与融合备注 |
| `amr_bringup/launch/stack_contract.launch` | 加载契约参数，可选 bluesea + 看门狗 |

---

## 7. 待与 mars 工程核对项

1. `/wheel_odom` 的 `frame_id` / `child_frame_id` 是否已是 `odom` / `base_link`。
2. IMU、电池对应的 **实际 ROS 话题名**（填入 `optional_topics`）。
3. `/lidar1/scan_filtered` 由哪个节点从 `laser_fan` 生成，滤波参数是否过滤 0 距离点（与 bluesea `zero_shift` 相关）。
4. 自动模式速度上限是 **STM32 固件** 还是 mars 参数；要到 1.5 m/s 需与 `teb_local_planner.yaml` 同步修改。

---

*下层协议细节见上传的《STM32 ↔ RK3588 串口通信协议文档》；雷达以 BlueSea **bluesea2** 为准，勿用 YDLIDAR 驱动配 LDS-50C-E 网口型号。*
