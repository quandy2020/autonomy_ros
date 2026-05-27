# autonomy_msgs

导航专用消息包：单点 `NavigatePose`、多点 `NavigateThrough`，以及任务状态与基础服务。

## Actions

| 文件 | 名称 | 说明 |
|------|------|------|
| `NavigatePose.action` | `/autonomy/navigate_pose` | 单点导航 |
| `NavigateThrough.action` | `/autonomy/navigate_through` | 顺序途经多点 |

## Services

| 文件 | 名称 |
|------|------|
| `CancelTask.srv` | `/autonomy/cancel_task` |
| `GetTaskStatus.srv` | `/autonomy/get_task_status` |
| `PauseTask.srv` | `/autonomy/pause_task` |
| `ResumeTask.srv` | `/autonomy/resume_task` |
| `TriggerEmergencyStop.srv` | `/autonomy/trigger_estop` |
| `SetInitialPose.srv` | `/autonomy/set_initial_pose` |

## Messages

| 文件 | 用途 |
|------|------|
| `TaskType.msg` | `IDLE` / `NAVIGATION` / `WAYPOINTS` |
| `TaskState.msg` | 任务生命周期 |
| `TaskStatus.msg` | `/autonomy/status` |
| `Event.msg` | `/autonomy/events` |
| `Error.msg` | 统一错误码 |
| `Waypoint.msg` | 多点导航路点 |
| `WaypointStatus.msg` | 各路点结果 |

## 构建

```bash
colcon build --packages-select autonomy_msgs
source install/setup.bash
ros2 interface show autonomy_msgs/action/NavigatePose
```
