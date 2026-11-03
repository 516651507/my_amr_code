# Web 控制与 APP 对接说明

## 架构

```text
浏览器 / 手机 APP
    │  HTTP REST（:8080）     WebSocket（:9090，可选 rosbridge）
    ▼
amr_web（本仓库）
    │  /cmd_vel（手动遥控）   /amr/load_mode（空载/负载）
    │  只读：/wheel_odom、/battery_state、/amr/load_mode
    ▼
amr_chassis / amr_navigation（自动仍走 /autoVel_cmd，导航时勿用 Web 抢 cmd_vel）
```

| 通道 | 用途 |
|------|------|
| **REST `http://<rk>:8080`** | APP 推荐：遥控、模式、负载档位、状态查询 |
| **rosbridge `ws://<rk>:9090`** | 高级：RViz 类、自定义订阅 `/map`、导航 goal |

## 启动

```bash
source devel/setup.bash
roslaunch amr_bringup full_robot.launch start_bluesea:=true
roslaunch amr_web web_control.launch
```

浏览器打开：`http://<RK3588_IP>:8080/`

## REST API（APP 契约）

| 方法 | 路径 |  body | 说明 |
|------|------|--------|------|
| GET | `/api/status` | — | 里程计、电池、负载模式、最大线速度 |
| POST | `/api/cmd_vel` | `{"linear":0.2,"angular":0.0}` | **手动**速度，发 `/cmd_vel` |
| POST | `/api/cmd_vel/stop` | — | 零速 |
| POST | `/api/load_mode` | `{"mode":"empty"}` 或 `"loaded"` | 空载 1.5 / 负载 0.8 m/s |
| GET | `/api/load_mode` | — | 当前档位 |

**安全约定（请 APP 遵守）：**

1. **自动导航运行中** 不要持续发 `/api/cmd_vel`（会与 `/autoVel_cmd` 抢底盘；chassis 有 auto 优先，但仍可能抖动）。
2. APP 应提供 **「手动 / 自动」** 开关；自动模式只显示状态，不发 cmd_vel。
3. 松手或断线应调用 **stop**（Web 页已实现 timeout 停）。

## APP 开发建议

- **Phase 1**：HTTP REST + 轮询 `/api/status`（最简单，跨 iOS/Android/Flutter）。
- **Phase 2**：WebSocket 订阅 rosbridge 的 `/map`、`/amr/fusion/scan` 做地图显示。
- **Phase 3**：`move_base_simple/goal` 通过 rosbridge 发导航目标（需地图与定位已就绪）。

## 依赖

```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-topic-tools
pip3 install flask  # 若 apt 无 python3-flask
```

`amr_web` 的 HTTP 服务默认 **0.0.0.0:8080**，局域网 APP 可直连 RK3588。
