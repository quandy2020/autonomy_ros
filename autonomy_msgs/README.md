# autonomy_msgs

对外部系统（App、讲解中控、云端）暴露的自主导航接口，设计参考 [nav2_msgs](https://github.com/ros-navigation/navigation2/tree/main/nav2_msgs)。

**Action / Service 字段说明与 ROS 2 命令示例**：[autonomy_ros/docs/external_commands.md](../autonomy_ros/docs/external_commands.md)

**目标场景：展厅讲解机器人**（导览、展点讲解、跟随观众、回充、馆员遥操）。

接口文件内以 `#` 注释说明每个字段；可用 `ros2 interface show autonomy_msgs/...` 查看生成后的类型。

---

## 展厅业务映射

| 现场能力 | 推荐接口 |
|----------|----------|
| 按路线讲解各展点 | **`GuidedTour`** action |
| 到展点播放讲解词 | `ExhibitPoint.narration_id` + `Event.ARRIVED_AT_EXHIBIT` |
| 等观众听完再走 | `wait_for_continue` + **`ContinueTour`** srv |
| 馆员暂停 / 继续 | **`PauseTask`** / **`ResumeTask`** |
| 跳过某展点 | **`SkipToExhibit`** |
| 散客跟随讲解 | **`Follow`** action（`use_target_pose`） |
| 馆员接管 | **`Teleop`** / **`SetTeleopMode`** |
| 闭馆回充 | **`Dock`** action |
| 低电自动回充 | `GuidedTour.auto_dock_on_low_battery` + **`BatteryStatus`** |
| 急停 | **`TriggerEmergencyStop`** |
| 任务被抢占 | `Event.TASK_PREEMPTED` |
| 换厅重定位 | **`SetInitialPose`** |
| 状态大屏 | **`TaskStatus`** + **`Event`** topic |

---

## 消息 `msg/`

| 文件 | 说明 |
|------|------|
| `Error.msg` | 统一错误码（9xxx 系统 / 1xxx 导航 / 6xxx 导览等） |
| `TaskType.msg` | 任务类型枚举，含 `GUIDED_TOUR` |
| `TaskState.msg` | 任务生命周期 |
| `TaskStatus.msg` | 周期状态（含电量、展点、位姿） |
| `Event.msg` | 边沿事件（到展点、抢占、急停等） |
| `BatteryStatus.msg` | 电量与低电标志 |
| `Waypoint.msg` | 多点导航航点 |
| `WaypointStatus.msg` | 航点/展点执行结果 |
| `ExhibitPoint.msg` | 展点位姿 + 讲解 + 停留策略 |
| `FollowTarget.msg` | 跟随目标描述 |
| `DockStation.msg` | 充电桩位姿定义 |

---

## 动作 `action/`

| 文件 | 服务端名 | 说明 |
|------|----------|------|
| `GuidedTour.action` | `/autonomy/guided_tour` | **展厅主流程** |
| `NavigatePose.action` | `/autonomy/navigate_pose` | 单点导航 |
| `NavigateThrough.action` | `/autonomy/navigate_through` | 多点顺序导航 |
| `Follow.action` | `/autonomy/follow` | 跟随 |
| `Dock.action` | `/autonomy/dock` | 回充 |
| `Teleop.action` | `/autonomy/teleop` | 遥操会话 |

Action 三段：`--- goal ---` / `--- result ---` / `--- feedback ---`。

---

## 服务 `srv/`

| 文件 | 服务端名 |
|------|----------|
| `CancelTask.srv` | `/autonomy/cancel_task` |
| `GetTaskStatus.srv` | `/autonomy/get_task_status` |
| `PauseTask.srv` | `/autonomy/pause_task` |
| `ResumeTask.srv` | `/autonomy/resume_task` |
| `ContinueTour.srv` | `/autonomy/continue_tour` |
| `SkipToExhibit.srv` | `/autonomy/skip_to_exhibit` |
| `TriggerEmergencyStop.srv` | `/autonomy/trigger_estop` |
| `SetInitialPose.srv` | `/autonomy/set_initial_pose` |
| `SetTeleopMode.srv` | `/autonomy/set_teleop_mode` |
| `ListDocks.srv` | `/autonomy/list_docks` |

---

## 推荐 Topic（由 autonomy_ros 发布）

| Topic | 类型 | 说明 |
|-------|------|------|
| `/autonomy/status` | `TaskStatus` | ~5 Hz |
| `/autonomy/events` | `Event` | 边沿事件 |
| `/autonomy/battery` | `BatteryStatus` | 电量 |
| `/cmd_vel` | `TwistStamped` | 遥操或控制器输出 |

---

## 与 AV/中控协作

```
GuidedTour → 机器人到展点
    → Event(ARRIVED_AT_EXHIBIT, narration_id)
        → 中控按 narration_id 播 TTS/视频
    → wait_for_continue ? ContinueTour : dwell 超时
    → 下一展点 → Event(TOUR_COMPLETED)
```

---

## 编译

```bash
colcon build --packages-select autonomy_msgs
source install/setup.bash
ros2 interface show autonomy_msgs/action/GuidedTour
ros2 interface show autonomy_msgs/msg/Event
```

修改 `.msg` / `.srv` / `.action` 后需重新编译本包及依赖它的 `autonomy_ros`。
