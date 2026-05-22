# 外部指令 API

本文档描述展厅讲解机器人对外暴露的 **Action**（长时任务）与 **Service**（短时命令）。  
消息定义见 `autonomy_msgs`；服务端实现见 `autonomy_ros/command/command_interface.cpp`。

设计参考 [nav2_msgs](https://github.com/ros-navigation/navigation2/tree/main/nav2_msgs)。

---

## 命名空间

所有接口挂在 `autonomy_node` 下，统一前缀 **`autonomy/`**：

| 类型 | 命名示例 |
|------|----------|
| Action | `/autonomy/navigate_pose` |
| Service | `/autonomy/cancel_task` |
| Topic | `/autonomy/status`、`/autonomy/events` |

---

## 配套 Topic（非 Action/Srv，建议订阅）

| Topic | 消息类型 | 方向 | 说明 |
|-------|----------|------|------|
| `/autonomy/status` | `autonomy_msgs/msg/TaskStatus` | 发布 ~5 Hz | 当前任务状态、进度、展点讲解 ID |
| `/autonomy/events` | `autonomy_msgs/msg/Event` | 事件 | 到展点、导览开始/结束、急停、抢占等 |
| `/autonomy/battery` | `autonomy_msgs/msg/BatteryStatus` | 发布 ~5 Hz | 电量（参数 `task.battery_percent`） |
| `/init_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 订阅 | RViz 重定位；转发到 `/initialpose` |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 发布 | `SetInitialPose` 服务或 `/init_pose` 触发 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 订阅 | RViz 2D Goal Pose；启动导航任务 |

---

## Action 一览

| Action 类型 | 服务端名称 | 场景 |
|-------------|------------|------|
| `NavigatePose` | `/autonomy/navigate_pose` | 单点导航 |
| `NavigateThrough` | `/autonomy/navigate_through` | 多点顺序导航 |
| `GuidedTour` | `/autonomy/guided_tour` | **展厅导览主流程** |
| `Follow` | `/autonomy/follow` | 跟随目标 |
| `Dock` | `/autonomy/dock` | 自动回充 |
| `Teleop` | `/autonomy/teleop` | 遥操会话 |

通用约定：

- **Goal** 中 `task_id` 建议由调用方生成（如 UUID），便于查询与幂等。
- **Result** 均含 `autonomy_msgs/msg/Error`（`error_code` + `error_msg`）。
- 执行中通过 **Feedback** 上报进度；终态见 `TaskState`（`SUCCEEDED` / `FAILED` / `CANCELED`）。

---

### NavigatePose — 单点导航

**类型**：`autonomy_msgs/action/NavigatePose`  
**参考**：Nav2 `NavigateToPose`

#### Goal

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `goal` | `geometry_msgs/PoseStamped` | 目标位姿 |
| `behavior_tree` | `string` | 行为树 XML（预留，当前未接 Nav2） |
| `use_start` | `bool` | 是否使用自定义起点 |
| `start` | `geometry_msgs/PoseStamped` | 起点（`use_start=true` 时） |
| `max_speed` | `float32` | 最大速度，`0` 表示默认 |

#### Result

| 字段 | 类型 | 说明 |
|------|------|------|
| `error` | `Error` | 错误信息 |
| `path` | `nav_msgs/Path` | 规划路径 |
| `navigation_time` | `duration` | 导航耗时 |

#### Feedback

| 字段 | 类型 | 说明 |
|------|------|------|
| `current_pose` | `PoseStamped` | 当前位姿 |
| `distance_remaining` | `float32` | 剩余距离 |
| `position_tracking_error` | `float32` | 位置跟踪误差 |
| `navigation_time` | `duration` | 已用时间 |
| `estimated_time_remaining` | `duration` | 预计剩余时间 |
| `number_of_recoveries` | `uint16` | 恢复行为次数 |

命令行示例见文末 **[ROS2 命令行测试与调试](#ros2-命令行测试与调试)**。

---

### NavigateThrough — 多点导航

**类型**：`autonomy_msgs/action/NavigateThrough`  
**参考**：Nav2 `FollowWaypoints`

#### Goal

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `waypoints` | `Waypoint[]` | 航点列表（见 `autonomy_msgs/msg/Waypoint`） |
| `number_of_loops` | `uint32` | 循环次数，默认 `1` |
| `start_index` | `uint32` | 起始航点索引 |
| `stop_on_failure` | `bool` | 失败时是否停止 |
| `behavior_tree` | `string` | 行为树（预留） |

`Waypoint` 主要字段：`id`、`label`、`pose`、`wait_duration`、`narration_id`、`wait_for_continue`。

#### Result

| 字段 | 类型 |
|------|------|
| `error` | `Error` |
| `waypoint_statuses` | `WaypointStatus[]` |
| `completed_count` | `uint32` |

#### Feedback

| 字段 | 类型 |
|------|------|
| `current_waypoint_index` | `uint32` |
| `waypoint_statuses` | `WaypointStatus[]` |
| `distance_remaining` | `float32` |
| `current_pose` | `PoseStamped` |

---

### GuidedTour — 展厅导览（推荐主入口）

**类型**：`autonomy_msgs/action/GuidedTour`

#### Goal

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `tour_id` | `string` | 路线 ID |
| `tour_name` | `string` | 路线名称（展示用） |
| `exhibits` | `ExhibitPoint[]` | 展点列表 |
| `number_of_loops` | `uint32` | 循环圈数 |
| `start_exhibit_index` | `uint32` | 起始展点 |
| `return_to_dock_on_complete` | `bool` | 结束后是否回充 |
| `auto_dock_on_low_battery` | `bool` | 低电是否回充（预留策略） |
| `low_battery_threshold` | `float32` | 低电阈值 (%) |
| `cruise_speed` | `float32` | 展厅巡航速度，`0` 为默认 |

`ExhibitPoint` 主要字段：

| 字段 | 说明 |
|------|------|
| `exhibit_id` / `exhibit_name` | 展点标识 |
| `narration_id` | 播控/TTS 内容 ID |
| `pose` | 展点位姿 |
| `dwell_duration` | 到点后自动停留秒数 |
| `wait_for_continue` | `true` 时需调用 `ContinueTour` 才离开 |
| `xy_tolerance` / `yaw_tolerance` | 到达判定 |

#### Result

| 字段 | 类型 |
|------|------|
| `error` | `Error` |
| `exhibit_statuses` | `WaypointStatus[]` |
| `completed_exhibits` | `uint32` |
| `tour_duration` | `duration` |

#### Feedback

| 字段 | 类型 | 说明 |
|------|------|------|
| `tour_id` | `string` | 当前路线 |
| `current_exhibit_index` | `uint32` | 当前展点索引 |
| `current_exhibit_id` | `string` | 当前展点 ID |
| `current_narration_id` | `string` | **到点时播控应播放的内容 ID** |
| `state` | `TaskState` | 任务状态 |
| `progress` | `float32` | 进度 0~1 |
| `waiting_for_continue` | `bool` | 是否在等「下一步」 |
| `current_pose` | `PoseStamped` | 当前位姿 |

#### 协作流程

```text
下发 GuidedTour
  → 机器人到展点 → /autonomy/events (ARRIVED_AT_EXHIBIT, narration_id=xxx)
  → 中控播放讲解
  → wait_for_continue ? 调用 /autonomy/continue_tour : 等待 dwell_duration
  → 下一展点 → … → TOUR_COMPLETED
```

---

### Follow — 跟随

**类型**：`autonomy_msgs/action/Follow`

#### Goal

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `target` | `FollowTarget` | 目标（ID 或位姿，见 `FollowTarget.msg`） |
| `time_allowance` | `duration` | 最长跟随时间，`0` 表示不限 |

#### Result / Feedback

| 部分 | 主要字段 |
|------|----------|
| Result | `error`, `follow_duration` |
| Feedback | `active_target`, `distance_to_target`, `current_pose`, `state` |

---

### Dock — 回充

**类型**：`autonomy_msgs/action/Dock`  
**参考**：Nav2 `DockRobot`

#### Goal

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `use_dock_id` | `bool` | 使用 `dock_id` 或 `dock` 字段 |
| `dock_id` | `string` | 充电桩 ID（需在系统内配置） |
| `dock` | `DockStation` | 完整充电桩描述 |
| `navigate_to_staging_pose` | `bool` | 是否先导航到 staging 位姿 |
| `max_staging_time` | `float32` | 导航超时（秒） |

#### Result

| 字段 | 类型 |
|------|------|
| `error` | `Error` |
| `success` | `bool` |
| `charging` | `bool` |
| `num_retries` | `uint16` |
| `docking_time` | `duration` |

#### Feedback `state` 枚举

| 值 | 含义 |
|----|------|
| `STAGING` (1) | 前往停靠点 |
| `PERCEPTION` (2) | 感知充电桩 |
| `ALIGNING` (3) | 对准 |
| `CHARGING` (4) | 充电中 |
| `RETRY` (5) | 重试 |

---

### Teleop — 遥操会话

**类型**：`autonomy_msgs/action/Teleop`  
**参考**：Nav2 `AssistedTeleop`

#### Goal

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `time_allowance` | `duration` | 会话时长，`0` 表示直到取消 |
| `preempt_other_tasks` | `bool` | 是否抢占其他任务 |
| `max_linear_speed` | `float32` | 线速度上限 |
| `max_angular_speed` | `float32` | 角速度上限 |

速度指令约定：由外部发布 `/cmd_vel`（`geometry_msgs/TwistStamped`），与仿真桥接一致。

#### Result / Feedback

| 部分 | 主要字段 |
|------|----------|
| Result | `error`, `total_elapsed_time` |
| Feedback | `current_teleop_duration`, `last_cmd`, `state` |

---

## Service 一览

| Service 类型 | 服务端名称 | 说明 |
|--------------|------------|------|
| `CancelTask` | `/autonomy/cancel_task` | 取消任务 |
| `GetTaskStatus` | `/autonomy/get_task_status` | 查询状态 |
| `PauseTask` | `/autonomy/pause_task` | 暂停 |
| `ResumeTask` | `/autonomy/resume_task` | 继续 |
| `ContinueTour` | `/autonomy/continue_tour` | 展点「下一步」 |
| `SkipToExhibit` | `/autonomy/skip_to_exhibit` | 跳过/跳转展点 |
| `TriggerEmergencyStop` | `/autonomy/trigger_estop` | 急停 / 解除 |
| `SetInitialPose` | `/autonomy/set_initial_pose` | 重定位 |
| `SetTeleopMode` | `/autonomy/set_teleop_mode` | 快速开关遥操 |
| `ListDocks` | `/autonomy/list_docks` | 充电桩列表 |

---

### CancelTask

| | 字段 | 类型 | 说明 |
|---|------|------|------|
| **Request** | `task_id` | `string` | 空字符串表示取消当前活动任务 |
| | `task_type` | `TaskType` | 可选过滤 |
| | `cancel_all` | `bool` | `true` 强制取消 |
| **Response** | `success` | `bool` | |
| | `error` | `Error` | |
| | `status` | `TaskStatus` | 取消后状态 |

命令行示例见文末 **[ROS2 命令行测试与调试](#ros2-命令行测试与调试)**。

---

### GetTaskStatus

| | 字段 | 说明 |
|---|------|------|
| **Request** | `task_id`：空则查当前任务；非空须与活动任务 `task_id` 一致 |
| **Response** | `success`、`error`、`status`（`TaskStatus`） |

---

### PauseTask / ResumeTask

**PauseTask Request**：`task_id`（可空）、`reason`  
**ResumeTask Request**：`task_id`  

**Response**：`success`、`error`、`status`  

暂停时控制器停止输出速度；继续后恢复（急停未解除时除外）。

---

### ContinueTour

在 `GuidedTour` 某展点 `wait_for_continue=true` 时，观众或馆员听完讲解后调用。

| | 字段 |
|---|------|
| **Request** | `tour_id`、`task_id`（可选） |
| **Response** | `success`、`error`、`status` |

---

### SkipToExhibit

导览过程中跳转到指定展点。

| | 字段 | 说明 |
|---|------|------|
| **Request** | `exhibit_id`（优先）或 `exhibit_index` |
| **Response** | `success`、`error`、`status` |

---

### TriggerEmergencyStop

| | 字段 | 说明 |
|---|------|------|
| **Request** | `engage`：`true` 急停，`false` 解除 |
| | `reason`：原因说明 |
| **Response** | `success`、`error`、`status` |

急停会发布 `Event.EMERGENCY_STOP`，并停止 `cmd_vel` 输出。

---

### SetInitialPose

| | 字段 |
|---|------|
| **Request** | `geometry_msgs/PoseWithCovarianceStamped pose` |
| **Response** | `success`、`error` |

发布到 `/initialpose`，用于换厅或推车后重定位。

---

### SetTeleopMode

不启动 `Teleop` Action 时快速切换遥操。

| | 字段 | 说明 |
|---|------|------|
| **Request** | `enable`、`max_linear_speed`、`max_angular_speed`、`preempt_other_tasks` |
| **Response** | `success`、`error`、`status` |

---

### ListDocks

| | 字段 |
|---|------|
| **Request** | （空） |
| **Response** | `success`、`docks`（`DockStation[]`）、`error` |

---

## 公共消息类型

### TaskType

| 常量 | 值 | 含义 |
|------|-----|------|
| `IDLE` | 0 | 空闲 |
| `NAVIGATION` | 1 | 单点/导航类 |
| `FOLLOW` | 2 | 跟随 |
| `WAYPOINTS` | 3 | 多点 |
| `DOCK` | 4 | 回充 |
| `TELEOP` | 5 | 遥操 |
| `GUIDED_TOUR` | 6 | 展厅导览 |

### TaskState

| 常量 | 值 |
|------|-----|
| `UNKNOWN` | 0 |
| `ACCEPTED` | 1 |
| `RUNNING` | 2 |
| `SUCCEEDED` | 3 |
| `FAILED` | 4 |
| `CANCELED` | 5 |
| `PAUSED` | 6 |

### Error 错误码（节选）

| 码段 | 示例 |
|------|------|
| 系统 `9xxx` | `NOT_AVAILABLE`、`TIMEOUT`、`SAFETY_ESTOP` |
| 导航 `1xxx` | `NAV_GOAL_INVALID` |
| 跟随 `2xxx` | `FOLLOW_TARGET_LOST` |
| 航点 `3xxx` | `WP_NO_VALID_WAYPOINTS` |
| 回充 `4xxx` | `DOCK_NOT_FOUND` |
| 导览 `6xxx` | `TOUR_INVALID` |

完整定义见 `autonomy_msgs/msg/Error.msg`。

### Event 事件类型

| 常量 | 值 | 典型用途 |
|------|-----|----------|
| `ARRIVED_AT_EXHIBIT` | 1 | 开始播放讲解 |
| `LEFT_EXHIBIT` | 2 | 离开展点 |
| `TOUR_STARTED` | 3 | 导览开始 |
| `TOUR_COMPLETED` | 4 | 导览结束 |
| `TOUR_PAUSED` | 5 | 暂停 |
| `TOUR_RESUMED` | 6 | 恢复 |
| `EMERGENCY_STOP` | 8 | 急停 |
| `DOCK_STARTED` / `DOCK_COMPLETED` | 9 / 10 | 回充 |
| `TASK_PREEMPTED` | 12 | 任务被更高优先级抢占 |

---

## 任务 Muxer（互斥与抢占）

`TaskManager` 内置 `TaskMuxer`，**同一时刻仅一个任务占用执行槽**。新任务按优先级仲裁：

| 优先级（高→低） | TaskType |
|-----------------|----------|
| 100（`force_preempt`） | TELEOP（`preempt_other_tasks: true`） |
| 85 | TELEOP |
| 75 | DOCK |
| 65 | GUIDED_TOUR |
| 55 | WAYPOINTS |
| 50 | NAVIGATION |
| 45 | FOLLOW |

- 新任务优先级**更高**或 `Teleop`/`SetTeleopMode` 的 **`preempt_other_tasks: true`** → 抢占当前任务，旧 Action 以 `TASK_CONFLICT` / `TELEOP_PREEMPTED` 结束。  
- 同级或更低优先级 → `beginTask` 失败，`NOT_AVAILABLE` / `busy`。  
- `CancelTask` 会清空 Muxer 槽位。  

---

## 调用建议

1. **长时任务**用 Action（`GuidedTour`、`NavigatePose` 等），**短时控制**用 Service。  
2. 订阅 `/autonomy/status` 做状态栏，订阅 `/autonomy/events` 驱动播控。  
3. 展厅场景优先使用 **`GuidedTour` + `ContinueTour`**，不要仅用 `NavigateThrough` 代替（无讲解语义）。  
4. 馆员接管时发 **`Teleop`** 且 `preempt_other_tasks: true`，或先 `CancelTask`。  

---

## Python CLI（`autonomy_cmd.py`）

完整使用说明见 **[autonomy_cmd.md](autonomy_cmd.md)**。以下为摘要。

### 任务级配置 `tasks.json`

在 `scripts/examples/tasks.json` 中集中定义：

| 段 | 作用 |
|----|------|
| `defaults` | 默认 `frame`、`timeout`、`pause_between` 等 |
| `poses` | 命名位姿（`home`、`point_a`…），任务中按名引用 |
| `tasks` | 可复用任务：`navigate_pose` 支持 **`repeat`** 对同一 pose 重复导航 |
| `missions` | 任务序列，按顺序执行多个 `tasks` 或内联步骤 |

```bash
export AUTONOMY_TASKS_CONFIG=/path/to/tasks.json   # 可选，省略 --config

ros2 run autonomy_ros autonomy_cmd.py list-config
ros2 run autonomy_ros autonomy_cmd.py run-pose point_a --repeat 5
ros2 run autonomy_ros autonomy_cmd.py run-task goto_a_x3
ros2 run autonomy_ros autonomy_cmd.py run-mission demo_patrol --feedback
```

`navigate_through` 任务可用 `"poses": ["point_a", "point_b", "home"]`；`guided_tour` 用 `"goal_file": "guided_tour.json"`（相对配置文件目录）。

---

`autonomy_ros` 提供统一脚本，运行时解析子命令与参数，对应 `CommandInterface` 全部 Action/Service：

```bash
ros2 run autonomy_ros autonomy_cmd.py --help
ros2 run autonomy_ros autonomy_cmd.py navigate-pose --x 1.0 --y 0.5 --frame odom --feedback
ros2 run autonomy_ros autonomy_cmd.py cancel --all
ros2 run autonomy_ros autonomy_cmd.py guided-tour \
  --goal-json $(ros2 pkg prefix autonomy_ros)/share/autonomy_ros/scripts/examples/guided_tour.json
```

| 子命令 | 接口 |
|--------|------|
| `navigate-pose` | Action `/autonomy/navigate_pose` |
| `navigate-through` | Action `/autonomy/navigate_through`（`--goal-json`） |
| `guided-tour` | Action `/autonomy/guided_tour`（`--goal-json`） |
| `follow` | Action `/autonomy/follow` |
| `dock` | Action `/autonomy/dock` |
| `teleop` | Action `/autonomy/teleop` |
| `cancel` | Service `/autonomy/cancel_task` |
| `status` | Service `/autonomy/get_task_status` |
| `pause` / `resume` | Service pause/resume |
| `continue-tour` | Service `/autonomy/continue_tour` |
| `skip-exhibit` | Service `/autonomy/skip_to_exhibit` |
| `estop` | Service `/autonomy/trigger_estop` |
| `set-initial-pose` | Service `/autonomy/set_initial_pose` |
| `set-teleop-mode` | Service `/autonomy/set_teleop_mode` |
| `list-docks` | Service `/autonomy/list_docks` |
| `list-config` | 打印 `tasks.json` 中的 poses/tasks/missions |
| `run-pose` | 对命名 pose 导航（`--repeat N`） |
| `run-task` | 执行 `tasks` 中预定义任务 |
| `run-mission` | 执行 `missions` 序列 |

---

## ROS2 命令行测试与调试

以下命令在**已启动** `autonomy_stack`（或至少 `autonomy_node`）且已 `source install/setup.bash` 的前提下使用。仿真默认 `frame_id` 为 **`odom`**，目标坐标请按当前环境调整。

### 环境准备

```bash
# 工作空间
cd /path/to/autonomy_ros
colcon build --packages-up-to autonomy_ros
source install/setup.bash

# 启动仿真 + 自主栈（另开终端执行后续命令）
ros2 launch autonomy_ros autonomy_stack.launch.py use_sim_time:=true

# 确认节点与接口已注册
ros2 node list | grep autonomy
ros2 node info /autonomy_node
```

### 发现与诊断

```bash
# 列出 autonomy 相关接口
ros2 action list | grep autonomy
ros2 service list | grep autonomy
ros2 topic list | grep -E 'autonomy|plan|cmd_vel|odom|initialpose'

# 查看类型与字段（写 goal 前建议先执行）
ros2 interface list | grep autonomy_msgs
ros2 interface show autonomy_msgs/action/GuidedTour
ros2 interface show autonomy_msgs/srv/ContinueTour
ros2 interface show autonomy_msgs/msg/TaskStatus
ros2 interface show autonomy_msgs/msg/Event

# 查看 action 服务端状态
ros2 action info /autonomy/navigate_pose
ros2 action info /autonomy/guided_tour

# 查看 topic 频率与类型
ros2 topic info /autonomy/status
ros2 topic hz /autonomy/status
ros2 topic echo /autonomy/battery
ros2 topic info /plan
```

### Topic：订阅与发布

```bash
# 任务状态（约 5 Hz）
ros2 topic echo /autonomy/status

# 事件（到展点、导览开始/结束、急停等）
ros2 topic echo /autonomy/events

# 规划路径、速度输出（调试运动）
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel

# 里程计（确认仿真在跑）
ros2 topic echo /odom --once

# 遥操：在 Teleop / SetTeleopMode 开启后手动发速度（与栈内 cmd_vel 一致）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"

# 持续发布（Ctrl+C 停止）
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.15}, angular: {z: 0.0}}}" \
  --rate 10
```

### Action 通用操作

```bash
# 下发 goal 并打印 feedback（推荐调试时加 --feedback）
ros2 action send_goal /autonomy/navigate_pose autonomy_msgs/action/NavigatePose \
  "{task_id: 'nav_001', goal: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback

# 仅下发、不阻塞终端（后台跑，适合再开终端调 service）
ros2 action send_goal /autonomy/navigate_pose autonomy_msgs/action/NavigatePose \
  "{task_id: 'nav_002', goal: {header: {frame_id: 'odom'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" &

# 复杂 goal 可写入 YAML 文件后发送（避免引号转义）
# ros2 action send_goal /autonomy/guided_tour autonomy_msgs/action/GuidedTour guided_tour_goal.yaml --feedback

# 取消当前任务（比 action cancel 更简单，推荐）
ros2 service call /autonomy/cancel_task autonomy_msgs/srv/CancelTask \
  "{task_id: '', task_type: {value: 0}, cancel_all: true}"
```

---

### NavigatePose

```bash
ros2 action send_goal /autonomy/navigate_pose autonomy_msgs/action/NavigatePose \
  "{task_id: 'nav_001', goal: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}, behavior_tree: '', use_start: false, max_speed: 0.0}" \
  --feedback

# 指定起点（use_start: true）
ros2 action send_goal /autonomy/navigate_pose autonomy_msgs/action/NavigatePose \
  "{task_id: 'nav_start', use_start: true, start: {header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, goal: {header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

---

### NavigateThrough

```bash
ros2 action send_goal /autonomy/navigate_through autonomy_msgs/action/NavigateThrough \
  "{task_id: 'wp_001', waypoints: [
    {id: 'p1', label: '点1', pose: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, wait_duration: 2.0},
    {id: 'p2', label: '点2', pose: {header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}, wait_duration: 0.0, wait_for_continue: false}
  ], number_of_loops: 1, start_index: 0, stop_on_failure: true, behavior_tree: ''}" \
  --feedback
```

---

### GuidedTour

```bash
# 两展点：第二个 wait_for_continue，到点后需调用 continue_tour
ros2 action send_goal /autonomy/guided_tour autonomy_msgs/action/GuidedTour \
  "{task_id: 'tour_001', tour_id: 'hall_a', tour_name: 'A厅导览', exhibits: [
    {exhibit_id: 'ex1', exhibit_name: '展点1', narration_id: 'audio_001',
     pose: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
     dwell_duration: 8.0, wait_for_continue: false},
    {exhibit_id: 'ex2', exhibit_name: '展点2', narration_id: 'audio_002',
     pose: {header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}},
     dwell_duration: 30.0, wait_for_continue: true}
  ], number_of_loops: 1, start_from_beginning: true, start_exhibit_index: 0,
   return_to_dock_on_complete: false, auto_dock_on_low_battery: false,
   low_battery_threshold: 20.0, cruise_speed: 0.3}" \
  --feedback

# 到展点 2 且 waiting_for_continue 为 true 时，另开终端：
ros2 service call /autonomy/continue_tour autonomy_msgs/srv/ContinueTour \
  "{tour_id: 'hall_a', task_id: 'tour_001'}"
```

---

### Follow

```bash
# 跟随指定位姿目标
ros2 action send_goal /autonomy/follow autonomy_msgs/action/Follow \
  "{task_id: 'follow_001', target: {use_target_id: false, target_id: '', use_target_pose: true,
    target_pose: {header: {frame_id: 'odom'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}},
    follow_distance: 1.0, max_linear_speed: 0.5, target_frame: 'odom'},
   time_allowance: {sec: 120, nanosec: 0}}" \
  --feedback

# 跟随目标 ID（需实现侧支持对应跟踪源）
ros2 action send_goal /autonomy/follow autonomy_msgs/action/Follow \
  "{task_id: 'follow_002', target: {use_target_id: true, target_id: 'person_01', use_target_pose: false,
    follow_distance: 1.2, max_linear_speed: 0.4, target_frame: 'odom'},
   time_allowance: {sec: 0, nanosec: 0}}" \
  --feedback
```

---

### Dock

```bash
ros2 action send_goal /autonomy/dock autonomy_msgs/action/Dock \
  "{task_id: 'dock_001', use_dock_id: true, dock_id: 'dock_main', dock: {dock_id: '', dock_type: '', dock_pose: {header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, staging_pose: {header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}},
   navigate_to_staging_pose: true, max_staging_time: 300.0, min_battery_level: 0.0}" \
  --feedback

# 先查可用充电桩
ros2 service call /autonomy/list_docks autonomy_msgs/srv/ListDocks "{}"
```

---

### Teleop

```bash
# 开启遥操会话（60 秒），期间向 /cmd_vel 发速度
ros2 action send_goal /autonomy/teleop autonomy_msgs/action/Teleop \
  "{task_id: 'teleop_001', time_allowance: {sec: 60, nanosec: 0}, preempt_other_tasks: true,
   max_linear_speed: 0.5, max_angular_speed: 1.5}" \
  --feedback

# 不限时直到 cancel_task
ros2 action send_goal /autonomy/teleop autonomy_msgs/action/Teleop \
  "{task_id: 'teleop_002', time_allowance: {sec: 0, nanosec: 0}, preempt_other_tasks: true,
   max_linear_speed: 0.3, max_angular_speed: 1.0}" &
```

---

### Service：任务控制

```bash
# 取消当前任务
ros2 service call /autonomy/cancel_task autonomy_msgs/srv/CancelTask \
  "{task_id: '', task_type: {value: 0}, cancel_all: true}"

# 查询状态
ros2 service call /autonomy/get_task_status autonomy_msgs/srv/GetTaskStatus "{task_id: ''}"

# 暂停 / 恢复
ros2 service call /autonomy/pause_task autonomy_msgs/srv/PauseTask \
  "{task_id: '', reason: 'debug_pause'}"
ros2 service call /autonomy/resume_task autonomy_msgs/srv/ResumeTask "{task_id: ''}"

# 导览：下一步 / 跳展点
ros2 service call /autonomy/continue_tour autonomy_msgs/srv/ContinueTour \
  "{tour_id: 'hall_a', task_id: ''}"
ros2 service call /autonomy/skip_to_exhibit autonomy_msgs/srv/SkipToExhibit \
  "{tour_id: 'hall_a', exhibit_id: 'ex2', exhibit_index: 1}"

# 急停 / 解除
ros2 service call /autonomy/trigger_estop autonomy_msgs/srv/TriggerEmergencyStop \
  "{engage: true, reason: 'test_estop'}"
ros2 service call /autonomy/trigger_estop autonomy_msgs/srv/TriggerEmergencyStop \
  "{engage: false, reason: 'release_estop'}"
```

---

### Service：定位与遥操

```bash
# 重定位（发布到 /initialpose）
ros2 service call /autonomy/set_initial_pose autonomy_msgs/srv/SetInitialPose \
  "{pose: {header: {frame_id: 'odom', stamp: {sec: 0, nanosec: 0}},
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}},
          covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}}"

# 快速开关遥操（不启动 Teleop action）
ros2 service call /autonomy/set_teleop_mode autonomy_msgs/srv/SetTeleopMode \
  "{enable: true, max_linear_speed: 0.5, max_angular_speed: 1.5, preempt_other_tasks: true}"
ros2 service call /autonomy/set_teleop_mode autonomy_msgs/srv/SetTeleopMode \
  "{enable: false, max_linear_speed: 0.5, max_angular_speed: 1.5, preempt_other_tasks: false}"

# 充电桩列表
ros2 service call /autonomy/list_docks autonomy_msgs/srv/ListDocks "{}"
```

---

### 推荐调试流程（复制即用）

**终端 1** — 启动栈：

```bash
source install/setup.bash
ros2 launch autonomy_ros autonomy_stack.launch.py
```

**终端 2** — 监视状态与事件：

```bash
source install/setup.bash
ros2 topic echo /autonomy/status
# 或
ros2 topic echo /autonomy/events
```

**终端 3** — 下发导览并交互：

```bash
source install/setup.bash
# 1) 导览
ros2 action send_goal /autonomy/guided_tour autonomy_msgs/action/GuidedTour \
  "{task_id: 'debug_tour', tour_id: 'demo', tour_name: '调试路线', exhibits: [
    {exhibit_id: 'a', exhibit_name: 'A', narration_id: 'n1',
     pose: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
     dwell_duration: 5.0, wait_for_continue: true}
  ], number_of_loops: 1, return_to_dock_on_complete: false}" --feedback

# 2) 到点且等待时
ros2 service call /autonomy/continue_tour autonomy_msgs/srv/ContinueTour "{tour_id: 'demo', task_id: 'debug_tour'}"

# 3) 需要时取消
ros2 service call /autonomy/cancel_task autonomy_msgs/srv/CancelTask \
  "{task_id: '', task_type: {value: 0}, cancel_all: true}"
```

### TaskType 在 service 中的取值

| `task_type.value` | 常量 |
|-------------------|------|
| 0 | IDLE |
| 1 | NAVIGATION |
| 2 | FOLLOW |
| 3 | WAYPOINTS |
| 4 | DOCK |
| 5 | TELEOP |
| 6 | GUIDED_TOUR |

示例：仅取消导览类任务时可传 `task_type: {value: 6}`（是否生效取决于实现是否按类型过滤）。
