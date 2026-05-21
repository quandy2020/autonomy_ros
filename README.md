# autonomy_ros

面向**展厅讲解机器人**的 ROS 2 工作区：TurtleBot3 Gazebo 仿真 + 模块化自主栈 + 对外任务接口（`autonomy_msgs`）。

仿真资源已整合进本包（源自 [nav2_minimal_tb3_sim](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation/tree/main/nav2_minimal_tb3_sim)），无需单独 clone 仿真仓库。

---

## 功能概览

| 能力 | 说明 |
|------|------|
| 仿真 | Gazebo 单台 TB3 Waffle、激光、里程计、RealSense D435i |
| 导航栈（骨架） | map / planner / controller / visualization |
| 对外指令 | `autonomy_msgs` 的 Action / Service / Topic |
| 展厅导览 | `GuidedTour`：多展点 + 讲解 ID + 等待观众 |

---

## 工程结构

```
autonomy_ros/                    # 仓库根（colcon workspace 的 src 目录）
├── autonomy_msgs/               # 对外 API 定义，详见 autonomy_msgs/README.md
│   ├── msg/ action/ srv/
│   └── README.md
├── autonomy_ros/                # 主功能包
│   ├── include/autonomy_ros/
│   │   ├── autonomy_node.hpp
│   │   ├── task/task_manager.hpp
│   │   ├── command/command_interface.hpp
│   │   ├── map/map_manager.hpp
│   │   ├── planner/planner.hpp
│   │   ├── controller/controller.hpp
│   │   └── visualization/visualizer.hpp
│   ├── src/                     # 与 include 对应实现
│   ├── launch/
│   │   ├── tb3_simulator.launch.py
│   │   └── autonomy_stack.launch.py
│   ├── config/autonomy_params.yaml
│   ├── rviz/autonomy.rviz
│   ├── configs/                 # ros_gz_bridge
│   ├── worlds/ urdf/ models/
│   └── CMakeLists.txt
└── README.md
```

---

## 软件架构

```mermaid
flowchart TB
  subgraph external [外部系统]
    App[App / 中控 / 播控]
  end
  subgraph command [command]
    AS[Action Server]
    SS[Service]
  end
  subgraph task [task]
    TM[TaskManager]
  end
  subgraph stack [自主栈]
    PL[planner]
    CT[controller]
    MP[map]
    VZ[visualization]
  end
  subgraph sim [Gazebo + bridge]
    GZ[gz sim]
    TB3[TurtleBot3]
  end
  App --> AS
  App --> SS
  AS --> TM
  SS --> TM
  TM --> PL
  TM --> CT
  PL -->|plan| CT
  CT -->|cmd_vel| TB3
  GZ --> TB3
  TM -->|status events| App
```

| 模块 | 职责 |
|------|------|
| **command** | 响应 `autonomy_msgs` 全部 Action / Service |
| **task** | 任务状态、暂停/急停、发布 `autonomy/status` 与 `autonomy/events` |
| **planner** | 目标点 → 直线路径 `/plan` |
| **controller** | 路径跟踪 → `/cmd_vel` |
| **map** | 订阅 `/map` |
| **visualization** | RViz Marker `/visualization/markers` |

---

## 依赖

```bash
# 以 Jazzy/Humble 为例，请替换 $ROS_DISTRO
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-ros-gz-interfaces \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-xacro
```

---

## 编译

```bash
cd /path/to/your_ws
# 将本仓库置于 src/ 下，例如 src/autonomy_ros
colcon build --symlink-install --packages-up-to autonomy_ros
source install/setup.bash
```

---

## 启动

### 完整栈（推荐）

仿真 + 自主栈 + RViz：

```bash
ros2 launch autonomy_ros autonomy_stack.launch.py
```

### 仅仿真

```bash
ros2 launch autonomy_ros tb3_simulator.launch.py
# 无 Gazebo GUI：
ros2 launch autonomy_ros tb3_simulator.launch.py headless:=True
```

### 模块开关

编辑 `autonomy_ros/config/autonomy_params.yaml`：

| 参数 | 默认 | 说明 |
|------|------|------|
| `enable_map` | true | 订阅地图 |
| `enable_planner` | true | 路径规划 |
| `enable_controller` | true | 速度控制 |
| `enable_visualization` | true | RViz 标记 |
| `enable_command` | true | 对外 Action/Srv（需 planner+controller） |

---

## 对外接口（`autonomy/*`）

- **命令说明（Action / Service 字段与示例）**：[autonomy_ros/docs/external_commands.md](autonomy_ros/docs/external_commands.md)
- **消息定义**： [autonomy_msgs/README.md](autonomy_msgs/README.md)

### Action（长时任务）

| Action | 用途 |
|--------|------|
| `/autonomy/guided_tour` | **展厅导览**（主流程） |
| `/autonomy/navigate_pose` | 单点导航 |
| `/autonomy/navigate_through` | 多点导航 |
| `/autonomy/follow` | 跟随 |
| `/autonomy/dock` | 回充 |
| `/autonomy/teleop` | 遥操会话 |

### Service（短时命令）

| Service | 用途 |
|---------|------|
| `/autonomy/continue_tour` | 展点讲解后「下一步」 |
| `/autonomy/pause_task` / `resume_task` | 暂停 / 继续 |
| `/autonomy/skip_to_exhibit` | 跳过展点 |
| `/autonomy/cancel_task` | 取消任务 |
| `/autonomy/get_task_status` | 查询状态 |
| `/autonomy/trigger_estop` | 急停 |
| `/autonomy/set_initial_pose` | 重定位 |
| `/autonomy/set_teleop_mode` | 开关遥操 |
| `/autonomy/list_docks` | 充电桩列表 |

### Topic

| Topic | 类型 |
|-------|------|
| `/autonomy/status` | `autonomy_msgs/msg/TaskStatus` |
| `/autonomy/events` | `autonomy_msgs/msg/Event` |
| `/autonomy/battery` | `autonomy_msgs/msg/BatteryStatus` |

### 示例：查询状态

```bash
ros2 topic echo /autonomy/status
ros2 service call /autonomy/get_task_status autonomy_msgs/srv/GetTaskStatus "{}"
```

### 示例：RViz 手动目标（调试）

在 RViz 用 **2D Goal Pose** 发布到 `/goal_pose`，可走 planner → controller 链路（不经过 Action）。

---

## 仿真传感器 Topic

| Topic | 类型 |
|-------|------|
| `/scan` | `sensor_msgs/LaserScan` |
| `/odom` | `nav_msgs/Odometry` |
| `/imu` | `sensor_msgs/Imu` |
| `/cmd_vel` | `geometry_msgs/TwistStamped`（订阅） |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` |

主要 TF：`odom` → `base_footprint` → `base_link` → `base_scan` / `camera_*`。

---

## 展厅导览协作流程

```
中控下发 GuidedTour
  → 机器人导航至展点
  → /autonomy/events 发布 ARRIVED_AT_EXHIBIT（含 narration_id）
  → 播控/TTS 播放讲解
  → wait_for_continue 时调用 continue_tour，或 dwell 超时自动下一站
  → 重复直至 TOUR_COMPLETED
  → 可选自动 Dock 回充
```

---

## 当前限制

- 规划为 **odom 系直线骨架**，无全局地图避障（未使用 `/map` 做碰撞检测）。
- `Follow` 的 `target_id` 跟踪未实现，仅支持 `use_target_pose`。
- `behavior_tree` 等 Nav2 专用字段为预留，当前忽略。
- Action 在独立线程执行；**TaskMuxer** 按优先级仲裁，高优先级可抢占低优先级任务。

---

## 致谢

- 仿真模型：[nav2_minimal_turtlebot_simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation)（Apache-2.0）
- 接口设计参考：[nav2_msgs](https://github.com/ros-navigation/navigation2/tree/main/nav2_msgs)
