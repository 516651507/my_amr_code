# 外参是什么意思？（laser_link、相机 与 base_link）

## base_link 是什么

**base_link** 是 ROS 里 **车体坐标系**，通常取 **底盘几何中心**（或后轴中心，需在团队内固定一种定义）。

导航、融合、TF 都假设：**机器人在平面上的位置和朝向** 就是 `base_link` 在 `odom` / `map` 里的位姿。

## laser_link 相对 base_link 指什么

**laser_link** 是 **2D 激光雷达的测量参考系**（旋转中心或光心，与 bluesea2 的 `frame_id` 一致）。

**外参** = 从 `base_link` 到 `laser_link` 的 **固定刚体变换**（平移 xyz + 旋转 rpy）：

- 例如雷达装在车顶正前方 0.35 m、高 0.25 m → 在 URDF 里写 `<origin xyz="0.35 0 0.25" rpy="0 0 0"/>`。
- 你之前若用 **单位变换**（雷达坐标 = 车体中心），表示 **尚未标定**，只适合调试；**建图、避障会系统性偏**。

**你需要提供或实测的**：雷达安装位置相对车体中心的 **前后 / 左右 / 高度（米）** 和 **安装角（度）**。

## 两台 DW2 相机相对 base_link 指什么

深度相机每个都有自己的 link（如 `camera_link`、`camera_depth_optical_frame`）。

- **camera_link**：相机外壳安装参考。
- **camera_depth_optical_frame**：**深度点云** 的坐标系（光学系：Z 前、X 右、Y 下）。

**外参** 同样是：`base_link` → 该相机 optical frame 的固定变换。

融合节点把点云变到 **base_link** 做避障；外参错了，障碍会在地图上 **整体平移/旋转**，出现 **虚假碰撞或漏检**。

**你需要提供或实测的**：每台相机相对车体中心的 **安装位置 + 俯仰角**（至少 xyz + 是否朝下照地）。

## 和 Web / APP 的关系

Web 遥控、APP 只发 **速度** 或 **导航目标**，**不直接改外参**。外参在 **URDF / static TF** 里配一次，标定好后长期不变。

## 当前仓库占位

`amr_description/urdf/amr.urdf.xacro` 里是 **占位值**。有实测数据后改 xacro 中的 `laser_xyz`、`camera_xyz`、`camera2_xyz` 即可，或发给我们合并进仓库。
