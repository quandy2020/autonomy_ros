# autonomy_ros 架构

`autonomy_ros` 是 **ROS 2 集成层**：将传感器、地图、TF、控制与导航 API 接到 `autonomy` 核心；规划、控制、代价地图与行为树在 `autonomy` 库内，由 `autonomy_ros::system::RosAutonomySystem` 组装。

---

## 目录结构

```
autonomy_ros/
├── include/autonomy_ros/
│   ├── system/
│   │   ├── ros_autonomy_system.hpp   # 入口编排
│   │   ├── options.hpp               # 参数结构体
│   │   └── constants.hpp             # 话题名、参数键
│   ├── bridge/
│   │   ├── tf_bridge.hpp
│   │   ├── map_bridge.hpp
│   │   ├── costmap_bridge.hpp
│   │   ├── sensor_bridge.hpp
│   │   └── actuation_bridge.hpp
│   ├── navigation/
│   │   ├── navigation_server.hpp     # Action / Service / RViz 话题
│   │   ├── task_manager.hpp
│   │   └── task_muxer.hpp
│   ├── viz/visualizer.hpp
│   ├── debug/diagnostics_publisher.hpp
│   └── conversions/                  # 导航栈用到的 ROS ↔ commsgs
├── src/                              # 与 include 镜像
├── config/
│   ├── parameters.yaml
│   ├── navigation_tasks.json
│   └── navigate_through_goal.json
├── launch/navigation_stack.launch.py
├── scripts/navigation_client.py
└── rviz/autonomy.rviz
```

可执行文件：`autonomy_node`（`src/main.cpp`）→ 创建 `RosAutonomySystem` 并 `spin`。

---

## 模块职责

| 模块 | 命名空间 | 职责 |
|------|----------|------|
| `RosAutonomySystem` | `autonomy_ros::system` | 加载 core、启动 bridge、创建 `NavigationServer` 与 `Visualizer` |
| `NavigationServer` | `autonomy_ros::navigation` | `NavigatePose` / `NavigateThrough` Action，任务控制 Service，`goal_pose` / `init_pose` |
| `TaskManager` | `autonomy_ros::navigation` | 任务状态发布，调用 `Autonomy::NavigateToPose` / `NavigateThroughPoses` |
| `TaskMuxer` | `autonomy_ros::navigation` | 单槽任务仲裁；`WAYPOINTS` 可抢占 `NAVIGATION` |
| `TfBridge` | `autonomy_ros::bridge` | `/tf`、`/tf_static` → core transform buffer |
| `MapBridge` | `autonomy_ros::bridge` | 静态地图 ROS 发布；可选外部 `/map` 订阅 |
| `CostmapBridge` | `autonomy_ros::bridge` | global/local costmap 定时发布 |
| `SensorBridge` | `autonomy_ros::bridge` | odom、激光、可选 point_cloud / range |
| `ActuationBridge` | `autonomy_ros::bridge` | `cmd_vel` 发布与控制定时器 |
| `Visualizer` | `autonomy_ros::viz` | `/plan`、`/navigation_goal`、`/robot_pose` |
| `DiagnosticsPublisher` | `autonomy_ros::debug` | `/diagnostics` 健康快照 |

---

## 数据流

```mermaid
flowchart LR
  RViz[RViz / CLI] --> NS[NavigationServer]
  NS --> TM[TaskManager]
  TM --> Core[autonomy::system::Autonomy]
  Sensors[Gazebo] --> SB[bridge::SensorBridge]
  SB --> Core
  Core --> Act[bridge::ActuationBridge]
  Act --> CMD[/cmd_vel]
  Core --> VZ[viz::Visualizer]
```

1. **导航入口**：Action / `goal_pose` → `NavigationServer` → `TaskManager` → core。
2. **BT 模式**（`autonomy.use_bt_navigation: true`）：`TaskScheduler` 驱动 planner / controller。
3. **直驱模式**：core 内 plan + `TickFollowPath`。
4. **传感器**：`SensorBridge` 将 odom / scan 注入 core 与 costmap；位姿同时驱动 `Visualizer`。
5. **控制**：定时 `TickControl` → `ActuationBridge` 发布 `cmd_vel`。

---

## 配置与 Launch

| 文件 | 用途 |
|------|------|
| `config/parameters.yaml` | `autonomy_node` 默认 ROS 参数 |
| `launch/navigation_stack.launch.py` | 仿真 + `autonomy_node` + 可选 RViz |

Launch 将 `core_config_directory` 写入 `autonomy.config_directory`（指向含 `autonomy.lua` 的 core 配置目录）。

关键参数（完整列表见 `parameters.yaml` 与 `options.cpp`）：

| 参数 | 说明 |
|------|------|
| `autonomy.use_bt_navigation` | 是否使用行为树导航 |
| `autonomy.planner.global_frame` | 规划坐标系（仿真默认 `odom`） |
| `autonomy.enable_scan_bridge` | 是否订阅激光 |
| `navigation.waypoint_timeout_sec` | 导航超时 |
| `navigation.goal_pose_topic` | RViz 目标话题名 |
| `visualization.frame_id` | 可视化默认 frame |

---

## 对外接口

详见 [external_commands.md](external_commands.md)。CLI 见 [navigation_client.md](navigation_client.md)。

---

## 相关文档

- [localization.md](localization.md) — 定位与 `map` 帧
- [conversions.md](conversions.md) — 消息转换层
- [../../autonomy_msgs/README.md](../../autonomy_msgs/README.md) — 消息定义
