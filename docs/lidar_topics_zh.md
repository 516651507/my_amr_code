# 雷达话题：`/scan` 和 `/lidar1/laser_fan` 有什么区别？

## 一句话

- **`/scan`**：bluesea2 **驱动默认**发布的 `sensor_msgs/LaserScan`，ROS 社区里最通用的名字。
- **`/lidar1/laser_fan`**：本项目约定的 **「原始 2D 扇区扫描」** 名字，表示「第 1 路雷达、尚未做 AMR 滤波」的数据，方便和滤波后话题区分。

**内容类型相同**，都是 `LaserScan`；差别主要是 **命名和流水线位置**，不是两种硬件协议。

## 推荐数据流（本仓库）

```text
bluesea2 (UDP, LDS-50C-E)
    发布 /scan
        │  topic_tools relay（或驱动 remap）
        ▼
    /lidar1/laser_fan          ← 原始扫描（命名空间 + 语义）
        │  amr_lidar/scan_filter_node
        ▼
    /lidar1/scan_filtered        ← 建图、amcl、融合避障只用这一条
```

## 为什么不用 `/scan` 直接建图？

可以，但不利于：

1. **多雷达** 时命名冲突（`/lidar1/...`、`/lidar2/...`）。
2. **滤波前后** 分开调试（RViz 同时看 raw / filtered）。
3. 和 **旧栈 / 文档** 对齐（brief 里就是 `laser_fan` → `scan_filtered`）。

## 网口

- RK：**eth1 = 192.168.158.15/24**
- 雷达：**192.168.158.98**，UDP **6668**
- 检查：`./scripts/check_lidar_network.sh`
