# 差速 AMR 导航栈（ROS1 Noetic）

彭飞这台差速 AMR 的应用层：建图、全局路径规划、雷达加双深度相机避障。

**下层（不在本仓库编译）**：STM32 底盘经 `mars_serial_server`（串口 **115200**，默认 brief 为 `/dev/ttyS4`）发布 `/wheel_odom`，并订阅 `/cmd_vel` 与 **`/autoVel_cmd`**；**蓝海 LDS-50C-E** 经 **bluesea2 UDP**（雷达 `192.168.158.98` → RK `eth1` `192.168.158.15:6668`）出 `/lidar1/laser_fan` → 现场滤波 → **`/lidar1/scan_filtered`**；**奥比中光大白 DW2 ×2** 在 **`~/camera_ws`** 出两路深度点云。

完整协议、差速换算与启动顺序见 **[docs/hardware_and_protocol_zh.md](docs/hardware_and_protocol_zh.md)**。

车体约 800 × 490 mm，轮径 **0.165 m**，轮距 **0.42 m**（与 mars 电机换算一致）。导航目标线速度 1.5 m/s。工作空间按机器人上的 `/home/linaro/catkin_ws` 来写，本仓库根目录本身就是一个 catkin 工作空间（里面有 `src/`）。

## 架构

```
/lidar1/scan_filtered --------+
/camera/depth/points_cam -----+--> sensor_fusion_node --> /amr/fusion/scan  --> costmap ObstacleLayer（打点+清障）
/camera2/depth/points_cam ----+                      \-> /amr/fusion/cloud --> FusionObstacleLayer（自研层）
                                                          |
/wheel_odom --> odom_tf_bridge --> TF odom -> base_link    |
                                                          v
建图:  gmapping(只用雷达) --> /map + TF map -> odom
导航:  map_server + amcl  --> TF map -> odom
       move_base
         global_planner   全局路径（地图坐标系）
         teb_local_planner 局部路径（差速，1.5 m/s）
              |
              v
       /amr/cmd_vel_raw --> cmd_smoother --> /autoVel_cmd
```

手动速度仍然走 `/cmd_vel`。自动模式只发 `/autoVel_cmd`，两边不混。

建图**不**使用相机。深度图里的地面和近处噪声会把栅格弄脏。相机只在导航时进入代价地图。

## 话题和坐标系

| 方向 | 名字 | 说明 |
| --- | --- | --- |
| 订阅 | `/lidar1/scan_filtered` | **LDS-50C-E** + bluesea2 滤波后的 `LaserScan`。原始扇区 `/lidar1/laser_fan` 由驱动/滤波节点处理，本栈不直接订阅 |
| 订阅 | `/wheel_odom` | `nav_msgs/Odometry` |
| 订阅 | `/camera/depth/points_cam` | **DW2** 相机 1 深度点云（`~/camera_ws`） |
| 订阅 | `/camera2/depth/points_cam` | **DW2** 相机 2 深度点云 |
| 发布 | `/autoVel_cmd` | 自动模式 `geometry_msgs/Twist` |
| 不发布 | `/cmd_vel` | 留给手动 |

坐标系：

| 坐标系 | 谁发布 |
| --- | --- |
| `map` → `odom` | 建图时 gmapping，导航时 amcl |
| `odom` → `base_link` | `odom_tf_bridge`（订阅 `/wheel_odom`）。底盘已经在发这条 TF 时把 `publish_wheel_odom_tf` 设为 false |
| `base_link` → `laser_link` | URDF，默认单位变换，和现有 `mars_tf_bridge` 一致 |
| `camera_link`、`camera_depth_optical_frame`、`camera2_*` | URDF 占位。外参没测，不要当成真值 |

点云的 `header.frame_id` 必须能 TF 到 `base_link`。DW2 驱动或 `mars_tf_bridge` 如果已经在发同名坐标系，启动时加：

```bash
publish_description_tf:=false
```

否则两套 TF 会打架。没有雷达、没有里程计、只想看坐标树时用占位 launch，不要和建图或导航一起开：

```bash
roslaunch amr_bringup stub_tf.launch
```

## 哪些是自己写的，哪些是现成 ROS 包

自己写的（集成和融合，不是闭源算法的仿品）：

| 包 / 节点 | 作用 |
| --- | --- |
| `amr_obstacle_fusion/sensor_fusion_node` | 把雷达端点和两路点云变到 `base_link`，按高度、距离、车体足迹过滤，体素压缩后发布 `/amr/fusion/scan` 和 `/amr/fusion/cloud`。同一角度保留更近的距离 |
| `amr_obstacle_fusion/FusionObstacleLayer` | `costmap_2d` 插件。订阅融合点云，把高度窗内的点标成致命障碍。过期不再画，静态地图由 StaticLayer 每周期重画，不会被这层擦掉 |
| `amr_navigation/cmd_smoother.py` | 线速度/角速度限幅、高速时压低角速度、小角速度死区、加速度限制。这是防蛇形的最后一层 |
| `amr_bringup/odom_tf_bridge.py` | `/wheel_odom` 转成 `odom` → `base_link` |
| `amr_mapping/mapping_assist.py` | 按 `map_server` 的 PGM/YAML 规则存图 |
| `amr_description` | 占位 URDF |

直接用 apt 里的包，没有改它们的源码：

| 包 | 用途 |
| --- | --- |
| `gmapping`（`slam_gmapping`） | 二维建图。Noetic 上 `ros-noetic-gmapping` 依赖少，RK3588 上比带 Ceres 的 slam_toolbox 好装。没有回环，大场景以后可以再换 |
| `map_server` | 存图、读图 |
| `amcl` | 在已有地图上定位，发布 `map` → `odom` |
| `move_base` | 把全局规划和局部规划串起来 |
| `global_planner` | 全局路径，Dijkstra |
| `teb_local_planner` | 差速局部规划 |
| `costmap_2d` 的 Static / Obstacle / Inflation | 静态地图、融合扫描的打点与射线清障、膨胀 |

## 硬件参数与契约文件

| 文件 | 说明 |
| --- | --- |
| `src/amr_bringup/config/machine.yaml` | 轮径/轮距、串口、LDS-50C-E 网口、DW2 话题 |
| `src/amr_bringup/config/bluesea_lds50ce.yaml` | bluesea2 UDP 模板（**勿用 YDLIDAR 驱动配网口 LDS-50C-E**） |
| `src/amr_bringup/config/orbbec_dw2.yaml` | 双 DW2 点云与融合备注 |
| `src/amr_bringup/launch/stack_contract.launch` | 加载契约 + 看门狗 |
| `scripts/check_lidar_network.sh` | `ping` 雷达 IP |

雷达驱动示例：复制 `launch/lidar_bluesea_lds50ce.driver.launch.example` 为 `lidar_bluesea_lds50ce.driver.launch`，安装 bluesea2 后 `start_driver:=true`。

## 依赖和编译

机器人是 Ubuntu 20.04 + ROS Noetic（aarch64 也可以）。开发机如果不是 20.04，不要在那上面 `catkin_make`。

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-catkin \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  ros-noetic-amcl \
  ros-noetic-move-base \
  ros-noetic-global-planner \
  ros-noetic-teb-local-planner \
  ros-noetic-costmap-2d \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  ros-noetic-diagnostic-msgs \
  ros-noetic-rviz

source /opt/ros/noetic/setup.bash
cd /home/linaro/my_amr_code          # 本仓库根目录
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

已经有 `/home/linaro/catkin_ws` 时，把 `src/amr_*` 链进去再编译：

```bash
ln -s /home/linaro/my_amr_code/src/amr_* /home/linaro/catkin_ws/src/
cd /home/linaro/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`src/CMakeLists.txt` 会包含 Noetic 的 catkin 顶层文件。如果这步报错，改成官方链接：

```bash
ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
```

## 建图

先确认雷达和里程计：

```bash
rostopic hz /lidar1/scan_filtered
rostopic echo -n 1 /wheel_odom
```

遥控器走 `/cmd_vel`，本栈建图时不发速度。

```bash
roslaunch amr_mapping mapping.launch
# 另开一个终端看图
roslaunch amr_mapping mapping.launch rviz:=true
```

TF 冲突时：

```bash
roslaunch amr_mapping mapping.launch publish_description_tf:=false publish_wheel_odom_tf:=false
```

慢慢开过要建的区域。存图：

```bash
rosrun amr_mapping save_map.sh site_a
# 或者
rosservice call /mapping_assist/save_map "{}"
```

默认写到 `src/amr_mapping/maps/amr_map.yaml` 和同名 `.pgm`。`mapping_assist` 没起来时，`save_map.sh` 会退回 `map_saver`。

加载一张已经有的图（导航也会做这件事）：

```bash
roslaunch amr_mapping load_map.launch map_file:=/绝对路径/site_a.yaml
```

`maps/placeholder.yaml` 是 8 m 见方、四周一圈障碍的空房间，只为了让导航节点能启动，不能当现场地图。

gmapping 参数在 `src/amr_mapping/config/gmapping.yaml`。粒子数默认 30，是为了 RK3588 单线程还能跟上。里程计打滑就把 `srr/srt/str/stt` 加大。

## 全局路径规划

`move_base` 的全局规划器是 `global_planner/GlobalPlanner`（Dijkstra，不是自写的搜索）。它在 `map` 坐标系的全局代价地图上出路径。全局代价地图由四层叠起来：

1. `StaticLayer`：`map_server` 的占据栅格
2. `ObstacleLayer`：融合扫描，既能标障碍也能沿射线把已经消失的障碍清掉
3. `FusionObstacleLayer`：融合点云里、高度 0.05–1.6 m 的点
4. `InflationLayer`：膨胀 0.55 m

未知栅格默认不可穿越（`allow_unknown: false`）。目标点用 RViz 的 `2D Nav Goal`，话题 `/move_base_simple/goal`。

全局规划器参数在 `src/amr_navigation/config/global_planner.yaml`。同一份参数写了两个命名空间（`GlobalPlanner` 和 `global_planner/GlobalPlanner`），插件实际读到哪个就用哪个。

## 导航（含避障）

```bash
roslaunch amr_navigation navigation.launch map_file:=$(rospack find amr_mapping)/maps/amr_map.yaml
# 需要 RViz：
roslaunch amr_navigation navigation.launch map_file:=.../amr_map.yaml rviz:=true
```

在 RViz 里：

1. Fixed Frame 用 `map`
2. `2D Pose Estimate` 把车摆到地图上的真实位置（`/initialpose`）
3. 粒子云收敛后再下 `2D Nav Goal`

没定位时全局规划会一直失败，这是预期行为。

自动速度在 `/autoVel_cmd`。底盘固件如果还把自动速度限在 0.25 m/s，规划器写成 1.5 也跑不起来，两边要一起放。

## 雷达 + 双相机避障

`sensor_fusion_node`（`src/amr_obstacle_fusion/config/fusion.yaml`）：

- 雷达射线变到 `base_link` 后直接进入融合扫描，不再按高度丢掉，避免水平雷达被高度窗滤光
- 相机点要求高度 0.08–1.50 m，用来去掉地面和天花板
- 落在车体足迹里的点丢掉（半长 0.42 m、半宽 0.27 m，比 800×490 的车身多留了 2 cm）
- 0.5° 一档，同一角度只留更近的距离
- 点云体素 8 cm，只压缩 `/amr/fusion/cloud`，不拿来抽稀扫描。抽稀会留出空档，代价地图会把空档当成自由空间清掉

代价地图足迹（规划用的多边形，几何中心在 `base_link`）：

```yaml
footprint: [[0.40, 0.245], [0.40, -0.245], [-0.40, -0.245], [-0.40, 0.245]]
```

升降后的货物如果超出这个框，把 `costmap_common.yaml` 和 TEB 的 `footprint_model/vertices` 一起加大。

融合扫描超过大约 0.5 s 不更新时，ObstacleLayer 会把代价地图标成过期，`move_base` 停止发速度。这是故意的。

## 1.5 m/s 和防蛇形

速度写在两处，改的时候一起改：

- `src/amr_navigation/config/teb_local_planner.yaml`：`max_vel_x: 1.5`，`max_vel_trans: 1.5`，倒车 0.15 m/s，角速度 1.0 rad/s，线加速度 0.8 m/s²
- `src/amr_navigation/config/cmd_smoother.yaml`：同样的线速度上限，再加加速度斜率

按 0.8 m/s² 从 1.5 m/s 刹停大约 1.9 m。局部代价地图 8 m × 8 m，TEB 向前看 5 m。近处障碍会把速度压到大约 0.35 倍（`obstacle_proximity_*`）。

蛇形主要是三种原因，参数按这个收的：

| 现象 | 处理 |
| --- | --- |
| 两条绕障拓扑来回跳 | `enable_homotopy_class_planning: false` |
| 不贴全局路径、自己左右找 | `global_plan_viapoint_sep: 0.35`，`weight_viapoint: 8`。这个间隔小于等于 0 时贴线权重无效 |
| 高速小角速度来回抖 | TEB 里 `weight_max_vel_theta: 4`、`weight_kinematics_forward_drive: 80`；`cmd_smoother` 在速度高时把角速度上限降到大约 40%，并且丢掉小于 0.04 rad/s 的角速度 |

规划话题超过 0.4 s 没有更新，平滑器按 1.5 m/s² 减速到 0。底盘侧仍应有自己的超时停车，不要只靠这一层。

局部代价地图如果在 RViz 里闪，先看两路相机的 TF，再把 `point_stride` 调到 4 或 5（默认 3，给 RK3588 留余量）。

## 包

| 包 | 内容 |
| --- | --- |
| `amr_robot` | 元包 |
| `amr_description` | 占位 URDF |
| `amr_bringup` | 里程计 TF、话题看门狗、`stub_tf.launch` |
| `amr_mapping` | 建图、存图、读图 |
| `amr_obstacle_fusion` | 融合节点和代价地图插件 |
| `amr_navigation` | amcl、move_base、TEB、速度平滑 |

不依赖 ROS 的逻辑检查（融合分桶、存图像素、速度平滑）可以在没有 Noetic 的机器上跑：

```bash
python3 scripts/verify_scaffold.py
```

`catkin_make` 必须在 Noetic 上做。
