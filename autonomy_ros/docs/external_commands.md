# 对外指令接口（autonomy_msgs）

导航专用 API：`NavigatePose`（单点）、`NavigateThrough`（多点），以及任务控制 Service / 状态 Topic。

完整定义见 [autonomy_msgs/README.md](../../autonomy_msgs/README.md)。

---

## Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/autonomy/status` | `TaskStatus` | 任务快照 ~5 Hz |
| `/autonomy/events` | `Event` | 抢占、暂停/恢复、急停等边沿事件 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 手动目标（抢占当前导航） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 重定位 |

---

## Actions

| Action | 名称 | 说明 |
|--------|------|------|
| `NavigatePose` | `/autonomy/navigate_pose` | 单点导航 |
| `NavigateThrough` | `/autonomy/navigate_through` | 顺序途经多点 |

### NavigatePose

**Goal**

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID（状态/取消用） |
| `goal` | `PoseStamped` | 目标位姿 |
| `behavior_tree` | `string` | 预留；非空返回 `NAV_GOAL_INVALID` |

**Result**：`error`、`path`、`navigation_time`

**Feedback**：`current_pose`、`distance_remaining`、`navigation_time`

### NavigateThrough

**Goal**

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `waypoints` | `Waypoint[]` | 路点列表 |
| `number_of_loops` | `uint32` | 循环次数（默认 1） |
| `start_index` | `uint32` | 起始路点索引 |
| `stop_on_failure` | `bool` | 任一路点失败是否中止 |
| `behavior_tree` | `string` | 预留 |

`Waypoint` 字段：`id`、`label`、`pose`、`wait_duration`（到点后等待秒数）。

**Result**：`error`、`waypoint_statuses`、`completed_count`

---

## Services

| Service | 名称 | 说明 |
|---------|------|------|
| `CancelTask` | `/autonomy/cancel_task` | 取消任务 |
| `GetTaskStatus` | `/autonomy/get_task_status` | 查询状态 |
| `PauseTask` | `/autonomy/pause_task` | 暂停导航 |
| `ResumeTask` | `/autonomy/resume_task` | 恢复导航 |
| `TriggerEmergencyStop` | `/autonomy/trigger_estop` | 急停 / 解除 |
| `SetInitialPose` | `/autonomy/set_initial_pose` | 设置初始位姿 |

---

## TaskStatus / Event

**TaskType**：`IDLE(0)`、`NAVIGATION(1)`、`WAYPOINTS(2)`

**TaskState**：`UNKNOWN`、`PENDING`、`RUNNING`、`SUCCEEDED`、`FAILED`、`CANCELED`

**Event 类型**：

| 常量 | 值 | 说明 |
|------|-----|------|
| `TASK_PREEMPTED` | 1 | 被更高优先级任务抢占 |
| `TASK_PAUSED` | 2 | 暂停 |
| `TASK_RESUMED` | 3 | 恢复 |
| `EMERGENCY_STOP` | 4 | 急停 |

---

## 快速示例

```bash
# 单点导航
ros2 action send_goal /autonomy/navigate_pose autonomy_msgs/action/NavigatePose \
  "{task_id: 'nav_001', goal: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}, behavior_tree: ''}" \
  --feedback

# 多点导航
ros2 action send_goal /autonomy/navigate_through autonomy_msgs/action/NavigateThrough \
  "{task_id: 'wp_001', waypoints: [
    {id: 'p1', label: '点1', pose: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, wait_duration: 2.0}
  ], number_of_loops: 1, start_index: 0, stop_on_failure: true, behavior_tree: ''}" \
  --feedback

# 暂停 / 取消
ros2 service call /autonomy/pause_task autonomy_msgs/srv/PauseTask "{task_id: '', reason: 'demo'}"
ros2 service call /autonomy/cancel_task autonomy_msgs/srv/CancelTask "{task_id: '', cancel_all: true}"

# CLI
ros2 run autonomy_ros navigation_client.py navigate-pose --x 1.0 --y 0.5 --feedback
ros2 run autonomy_ros navigation_client.py navigate-through \
  --goal-file $(ros2 pkg prefix autonomy_ros)/share/autonomy_ros/config/navigate_through_goal.json
```

---

## 任务仲裁

同一时刻仅一个导航任务。`WAYPOINTS` 优先级高于 `NAVIGATION`；RViz `goal_pose` 可抢占当前 Action。

新任务优先级更高时，旧 Action 以 `TASK_CONFLICT` 结束，并发布 `TASK_PREEMPTED` 事件。
