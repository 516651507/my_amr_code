# AMR 完整测试指南（小白版）

本文档面向**第一次接触 ROS 和本仓库**的同学，按顺序做即可。  
代码在 GitHub 分支 **`cursor/amr-nav-scaffold-dabf`**（合并 PR 前默认 **`master` 几乎是空的**，务必 checkout 正确分支）。

- 仓库：https://github.com/516651507/my_amr_code  
- 说明文档：[hardware_and_protocol_zh.md](hardware_and_protocol_zh.md)  
- 雷达话题：[lidar_topics_zh.md](lidar_topics_zh.md)  
- Web/APP：[web_and_app_zh.md](web_and_app_zh.md)

---

## 一、先认识这些词（不用背，查表即可）

| 词 | 含义 |
|----|------|
| **ROS / Noetic** | 机器人中间件；本车使用 **ROS1 Noetic**（Ubuntu 20.04） |
| **节点 (node)** | 一个正在运行的程序（底盘、雷达、导航各是一个或多个节点） |
| **话题 (topic)** | 节点之间传数据的“频道”，例如 `/wheel_odom` |
| **launch** | 一条命令同时启动多个节点和参数 |
| **TF** | 坐标系之间的位置关系（如 `odom` → `base_link` → `laser_link`） |
| **catkin_make** | 编译本仓库里的 C++ 包 |
| **source devel/setup.bash** | 让终端“认识”刚编译的包（**每个新终端都要做**） |

---

## 二、本车已配置的硬件（软件里写死的默认值）

| 设备 | 型号/说明 | 配置要点 |
|------|-----------|----------|
| 主控 | RK3588 + Ubuntu 20.04 | 跑 ROS |
| 底盘 MCU | STM32，mars 二进制串口协议 | **`/dev/ttyS4`**，115200 |
| 激光雷达 | 蓝海 **LDS-50C-E** | UDP；雷达 **192.168.158.98**，RK **eth1 = 192.168.158.15** |
| 深度相机 ×2 | 奥比 **大白 DW2** | USB；话题 `/camera/...`、`/camera2/...` |
| 车体 | 差速，约 800×490 mm | 轮径 0.165 m，轮距 0.42 m |

**尚未在配置文件里填好的（必须现场补）：**

- 两台 DW2 的 **USB 端口**（`src/amr_camera/config/dw2_dual.yaml` 或 Web `/debug`）  
- 雷达、相机相对车体的 **真实外参**（当前 URDF 是占位）  

---

## 三、测试前准备清单

### 3.1 电脑与系统

- [ ] 机器人上是 **Ubuntu 20.04**，已安装 **ROS Noetic**  
- [ ] 能上网（装依赖、克隆第三方驱动）  
- [ ] 至少 **2 个终端窗口**（一个看日志，一个发命令）  
- [ ] 可选：带显示屏的 RViz；或 SSH + 笔记本远程 RViz  

### 3.2 安全（必做）

- [ ] 测试区域 **无人、无障碍物会被撞坏**  
- [ ] 手边有 **急停** 或能立刻断电/断使能  
- [ ] 第一次动底盘用 **很低的速度**，人站在能拉住车的位置  
- [ ] 自动导航测试前，确认 **手动 `/cmd_vel` 能停**  

### 3.3 硬件接线

- [ ] 雷达上电，网线接到 RK 的 **eth1**（或你现场实际用的网口，IP 需与配置一致）  
- [ ] 底盘串口线接好，设备节点存在：`ls -l /dev/ttyS4`  
- [ ] 两台 DW2 USB 插入（测相机阶段再做）  

---

## 四、获取代码并编译（在 RK3588 上）

### 4.1 克隆与分支

```bash
cd ~
git clone https://github.com/516651507/my_amr_code.git
cd my_amr_code
git checkout cursor/amr-nav-scaffold-dabf
git pull
```

**成功标志：** `ls src` 能看到 `amr_chassis`、`amr_bringup` 等很多文件夹。

### 4.2 安装 ROS 依赖（首次）

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
  ros-noetic-rviz \
  ros-noetic-topic-tools \
  python3-pip

pip3 install pyyaml flask
```

### 4.3 第三方驱动（雷达 + 相机说明）

```bash
cd ~/my_amr_code
./scripts/install_third_party.sh
```

按脚本提示安装 **bluesea2**；Orbbec 驱动按脚本说明克隆/编译（常见路径 `~/camera_ws`）。

**雷达驱动 launch（必做一次）：**

```bash
cd ~/my_amr_code
cp src/amr_bringup/launch/lidar_bluesea_lds50ce.driver.launch.example \
   src/amr_bringup/launch/lidar_bluesea_lds50ce.driver.launch
```

打开 `lidar_bluesea_lds50ce.driver.launch`，按你本机 **bluesea2 包**里的 `udp_lidar.launch` 改 `include` 路径和参数名（不同版本键名可能略有差异，以 bluesea2 仓库为准）。

### 4.4 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/my_amr_code
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

**成功标志：** 最后一行类似 `Built target ...`，无 `error:`。

**失败时：** 复制**从第一个 error 起**的整段终端输出保存，便于排查。

**每个新终端都要执行：**

```bash
source /opt/ros/noetic/setup.bash
source ~/my_amr_code/devel/setup.bash
```

---

## 五、不装 ROS 也能做的检查（可选，在笔记本上）

用于确认下载的代码完整（不能代替上车测试）：

```bash
cd my_amr_code
bash scripts/verify_all.sh
```

**成功标志：** 最后一行 `=== verify_all ok ===`。

---

## 六、分阶段测试（推荐严格按顺序）

下面每一步都给出：**命令 → 看什么 → 成功标准 → 常见失败**。

> **说明：** 以下默认工作目录为 `~/my_amr_code`，且已 `source` 两个 setup.bash。

---

### 阶段 0：ROS 环境是否正常

```bash
echo $ROS_DISTRO
rospack find amr_chassis
```

| 项目 | 成功标准 |
|------|----------|
| `ROS_DISTRO` | 输出 `noetic` |
| `rospack find amr_chassis` | 输出 `.../my_amr_code/src/amr_chassis` |

**失败：** 未 `source devel/setup.bash` → 重新 source。

---

### 阶段 1：雷达网络

```bash
cd ~/my_amr_code
./scripts/check_lidar_network.sh
```

可选：若你的 IP 不是默认值：

```bash
./scripts/check_lidar_network.sh 192.168.158.98 192.168.158.15 eth1
```

| 项目 | 成功标准 |
|------|----------|
| ping | 脚本打印 `OK: ping 192.168.158.98 成功` |

**常见失败：**

- RK 未配置 `192.168.158.15/24` → 用 `ip addr` 检查 eth1  
- 雷达未上电、网线错口  
- 现场 IP 与文档不一致 → 改 `src/amr_bringup/config/machine.yaml` 并同步 bluesea 配置  

---

### 阶段 2：仅雷达 + 滤波（不动车）

**终端 1：**

```bash
source /opt/ros/noetic/setup.bash
source ~/my_amr_code/devel/setup.bash
roslaunch amr_bringup full_robot.launch start_bluesea:=true start_cameras:=false
```

> `start_bluesea:=true` 会尝试 include 你已复制的 `lidar_bluesea_lds50ce.driver.launch`。若尚未配置好，可先单独启动你熟悉的 bluesea2 launch，并保证有话题 **`/scan`**。

**终端 2：**

```bash
source /opt/ros/noetic/setup.bash
source ~/my_amr_code/devel/setup.bash
rostopic list | grep -E 'scan|lidar'
rostopic hz /scan
rostopic hz /lidar1/scan_filtered
rostopic echo -n 1 /lidar1/scan_filtered
```

| 话题 | 成功标准 |
|------|----------|
| `/scan` 或驱动原始话题 | `rostopic hz` 约 **10–25 Hz**（视驱动） |
| `/lidar1/scan_filtered` | 有稳定频率，`ranges` 数组非空 |

**话题关系（理解用）：**

```
bluesea2 → /scan → (relay) → /lidar1/laser_fan → scan_filter → /lidar1/scan_filtered
```

**常见失败：**

- `hz` 无输出 → 驱动未起、UDP IP 错、防火墙  
- 只有 `/scan` 没有 `/lidar1/scan_filtered` → `full_robot` 里 `start_lidar` 应为 true（默认 true）  

---

### 阶段 3：仅底盘（不启雷达也可）

**终端 1：**

```bash
roslaunch amr_chassis chassis.launch port:=/dev/ttyS4
```

若串口不是 ttyS4：

```bash
roslaunch amr_chassis chassis.launch port:=/dev/ttyUSB0
```

**终端 2：**

```bash
rostopic hz /wheel_odom
rostopic hz /imu/data
rostopic echo -n 1 /battery_state
rosrun tf tf_echo odom base_link
```

| 项目 | 成功标准 |
|------|----------|
| `/wheel_odom` | 推车或遥控时有频率；静止时 pose 变化小 |
| `/imu/data` | 有数据（频率依固件） |
| TF | `tf_echo` 能持续打印平移/旋转（推车时变化） |

**手动动一下（低速！）：**

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

按 `Ctrl+C` 停止发布。

| 项目 | 成功标准 |
|------|----------|
| 车 | 缓慢前进；若无反应检查固件使能、协议、线速度上限 |

**自动速度话题（只测收发，先低速）：**

```bash
rostopic pub -r 10 /autoVel_cmd geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

**常见失败：**

- `Permission denied` on `/dev/ttyS4` → `sudo usermod -aG dialout $USER` 后重新登录  
- 完全无 odom → 串口线、波特率 115200、MCU 是否运行 mars 协议  

---

### 阶段 4：底盘 + 雷达（全栈基础，仍不导航）

**终端 1：**

```bash
roslaunch amr_bringup full_robot.launch start_bluesea:=true start_cameras:=false start_web:=false
```

**终端 2 检查：**

```bash
rostopic hz /lidar1/scan_filtered
rostopic hz /wheel_odom
rosrun tf view_frames
# 会生成 frames.pdf，可用 scp 拷到 PC 查看；或：
rosrun rqt_tf_tree rqt_tf_tree
```

| 项目 | 成功标准 |
|------|----------|
| 同时有雷达与 odom | 两个 `hz` 都正常 |
| TF 树 | 至少存在 `odom → base_link → laser_link`（description 默认开启） |

---

### 阶段 5：双深度相机（DW2）

#### 5.1 查 USB

```bash
lsusb
lsusb -t
```

记录两路 DW2 的 **usb_port**（例如 `2-1.1`）。

#### 5.2 写入配置

编辑 `src/amr_camera/config/dw2_dual.yaml`，填 `camera1.usb_port` 与 `camera2.usb_port`（或之后用 Web `/debug` 写入）。

#### 5.3 启动

**终端 1：** 若阶段 4 已在跑，可只加相机；或重启：

```bash
roslaunch amr_bringup full_robot.launch start_bluesea:=true start_cameras:=true
```

**终端 2：**

```bash
rostopic hz /camera/depth/points_cam
rostopic hz /camera2/depth/points_cam
rostopic echo -n 1 /camera/depth/points_cam | head
```

| 项目 | 成功标准 |
|------|----------|
| 两路点云 | 均有 hz；`frame_id` 能在 TF 里变换到 `base_link`（若 RViz 点云跟车体对不上，是外参未标定，属预期） |

**常见失败：**

- 两路抢同一 USB → 必须填不同 `usb_port`  
- `orbbec_camera` 找不到 → 完成 `install_third_party.sh` 与 camera_ws 编译  

---

### 阶段 6：融合避障节点（导航前）

在 **阶段 4 或 5** 已运行的前提下，**终端 3：**

```bash
roslaunch amr_obstacle_fusion fusion.launch
```

**检查：**

```bash
rostopic hz /amr/fusion/scan
rostopic hz /amr/fusion/cloud
```

| 项目 | 成功标准 |
|------|----------|
| 融合输出 | 有频率；相机未开时 mainly 雷达；相机开后 cloud 更密 |

---

### 阶段 7：建图（gmapping）

**重要：** 仓库里的 `mapping_full.launch` **默认不打开 bluesea 驱动**（`start_bluesea:=false`）。小白请用下面 **两终端法**，避免“没雷达数据还在建图”。

**终端 1（底盘 + 雷达 + TF）：**

```bash
roslaunch amr_bringup full_robot.launch start_bluesea:=true start_cameras:=false
```

**终端 2（建图 + RViz）：**

```bash
roslaunch amr_mapping mapping.launch \
  publish_description_tf:=false \
  publish_wheel_odom_tf:=false \
  rviz:=true
```

**操作：**

1. 用 **遥控器** 发 `/cmd_vel`（不要用 `/autoVel_cmd` 建图阶段也可）  
2. **慢速** 把要建图的区域走一遍  
3. RViz Fixed Frame 选 **`map`**，添加 Map 显示，应看到栅格逐渐出现  

**检查：**

```bash
rostopic hz /map
rostopic hz /lidar1/scan_filtered
```

| 项目 | 成功标准 |
|------|----------|
| `/map` | 有更新；RViz 里地图随移动扩展 |

**存图：**

```bash
rosrun amr_mapping save_map.sh site_a
```

默认生成（若成功）：

- `src/amr_mapping/maps/amr_map.yaml`  
- `src/amr_mapping/maps/amr_map.pgm`  

或指定名：

```bash
rosrun amr_mapping save_map.sh site_a
# 实际文件名以 mapping_assist 参数为准，见终端日志
```

**成功标志：** 终端提示保存路径；`ls src/amr_mapping/maps/` 能看到新的 `.yaml` 和 `.pgm`。

---

### 阶段 8：导航（move_base + TEB + 避障）

使用 **阶段 7** 保存的地图（把路径换成你的文件）：

```bash
roslaunch amr_bringup navigation_full.launch \
  map_file:=$(rospack find amr_mapping)/maps/amr_map.yaml \
  start_bluesea:=true \
  start_cameras:=true
```

若还没有真地图，仅测节点能否起来（**不能当真导航**）：

```bash
roslaunch amr_bringup navigation_full.launch start_bluesea:=true
# 默认 placeholder 地图
```

**另开终端 + RViz（若 launch 未开 rviz）：**

```bash
roslaunch amr_navigation navigation.launch \
  map_file:=$(rospack find amr_mapping)/maps/amr_map.yaml \
  rviz:=true
```

（避免重复启动底盘时，只开带 rviz 的 navigation.launch 需理解节点是否重复——**推荐只用 navigation_full 一条命令**。）

**RViz 操作顺序：**

1. Fixed Frame：**`map`**  
2. **2D Pose Estimate**：在地图上点车真实位置和朝向 → 发布 `/initialpose`  
3. 等 AMCL 粒子收敛  
4. **2D Nav Goal**：点目标 → 规划路径  
5. 观察 **`/autoVel_cmd`**（不是 `/cmd_vel`）是否有输出  

```bash
rostopic echo /autoVel_cmd
rostopic hz /autoVel_cmd
```

| 项目 | 成功标准 |
|------|----------|
| 定位 | 粒子云与激光扫描大致重合 |
| 规划 | 出现全局/局部路径，车开始动（场地安全！） |
| 速度 | 空载模式约可达 1.5 m/s 配置；负载模式 Web 切 0.8 m/s |

**负载模式：**

```bash
# load_mode 在 full_robot 默认开启；也可单独：
roslaunch amr_control load_mode.launch
```

---

### 阶段 9：Web 与调试页

**终端 1：** 在 full_robot 基础上开 Web，或：

```bash
roslaunch amr_web web_control.launch
```

浏览器访问（把 IP 换成 RK 的局域网 IP）：

- 首页：`http://<RK3588_IP>:8080/`  
- 调试：`http://<RK3588_IP>:8080/debug`  

**检查 API：**

```bash
curl -s http://127.0.0.1:8080/api/status
```

| 项目 | 成功标准 |
|------|----------|
| `/api/status` | JSON 含 `load_mode`、`max_linear_speed` 等 |
| `/debug` | 能打开；修改相机 USB 配置后提示需重启 DW2 launch |

**注意：** Web **无登录密码**，仅适合局域网调试。

---

## 七、一键命令速查表

| 目的 | 命令 |
|------|------|
| 网络检查雷达 | `./scripts/check_lidar_network.sh` |
| 底盘+雷达 | `roslaunch amr_bringup full_robot.launch start_bluesea:=true` |
| + 相机 | 同上，`start_cameras:=true` |
| + Web | 同上，`start_web:=true` |
| 建图（推荐两终端） | 见 **阶段 7** |
| 导航 | `roslaunch amr_bringup navigation_full.launch map_file:=... start_bluesea:=true` |
| 本地无 ROS 自检 | `bash scripts/verify_all.sh` |

---

## 八、测试记录表（打印或复制填写）

| 日期 | 阶段 | 测试人 | 结果 OK/FAIL | 备注 |
|------|------|--------|--------------|------|
| | 1 雷达网络 | | | |
| | 2 雷达话题 | | | |
| | 3 底盘 | | | |
| | 4 雷达+底盘 | | | |
| | 5 双相机 | | | |
| | 6 融合 | | | |
| | 7 建图存图 | | | |
| | 8 导航 | | | |
| | 9 Web | | | |

---

## 九、常见问题 FAQ

### Q1：GitHub 主页看不到 `src` 文件夹？

默认分支是 **`master`**，几乎只有占位 readme。请切换到分支 **`cursor/amr-nav-scaffold-dabf`**，或查看 PR #1。

### Q2：`catkin_make` 报找不到 `costmap_2d` / `gmapping`？

未安装对应 apt 包 → 回到 **4.2** 安装 `ros-noetic-*`。

### Q3：`roslaunch` 报找不到 `lidar_bluesea_lds50ce.driver.launch`？

未复制 **driver.launch.example** → 见 **4.3**。

### Q4：有 `/scan` 没有 `/lidar1/scan_filtered`？

确认 `roslaunch amr_lidar lidar.launch` 或 `full_robot` 已启；`rostopic list` 看 `scan_filter_node` 是否在。

### Q5：导航不动，但 RViz 有路径？

- 是否在看 **`/autoVel_cmd`**（自动）而不是只发了 `/cmd_vel`  
- AMCL 是否未收敛 → 重设 **2D Pose Estimate**  
- 固件是否把速度限在 0.25 m/s → 需与规划 1.5 m/s 对齐  
- `rostopic echo /move_base/status` 看错误码  

### Q6：代价地图闪、CPU 100%？

减小相机负载：`fusion.yaml` 里 **`point_stride`** 改为 4 或 5；降低深度 FPS；检查相机 TF 是否抖动。

### Q7：建图地图很糊、漂移？

慢速行驶；检查 `/wheel_odom` 是否打滑；调 `gmapping.yaml` 粒子数与里程计噪声参数。

### Q8：两个终端都 source 了还是 `rospack find` 失败？

路径是否 `~/my_amr_code`；是否编译成功；`source devel/setup.bash` 是否在该仓库下执行。

---

## 十、建议的首次上车顺序（给完全小白）

1. 阶段 **1** 网络  
2. 阶段 **3** 底盘慢速  
3. 阶段 **2** 雷达  
4. 阶段 **4** 合并  
5. 阶段 **7** 建图（走一圈、存图）  
6. 阶段 **5、6** 相机与融合（若导航要带避障）  
7. 阶段 **8** 导航（先低速、小范围目标）  
8. 阶段 **9** Web  

---

## 十一、需要你提供给开发/标定的信息（测完后填写）

1. 雷达相对车体中心：前/后/左/右/高（米）+ 安装角（度）  
2. 相机 1、相机 2 同上  
3. 两路 DW2 的 `usb_port` 或序列号  
4. 自动模式是否必须硬件 AUTO 才允许动（当前软件：`/autoVel_cmd` 优先）  
5. `catkin_make` 或现场 roslaunch 的**完整报错日志**（若有）  

---

文档版本：与分支 `cursor/amr-nav-scaffold-dabf` 配套。若合并后分支名变化，以仓库 README 为准。
