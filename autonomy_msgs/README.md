# autonomy_msgs

导航专用消息包：单点 `NavigatePose`、多点 `NavigateThrough`，以及任务状态与基础服务。

## Actions

| 文件 | 名称 | 说明 |
|------|------|------|
| `NavigatePose.action` | `/navigate_pose` | 单点导航 |
| `NavigateThrough.action` | `/navigate_through` | 顺序途经多点 |

## Services

| 文件 | 名称 |
|------|------|
| `CancelTask.srv` | `/cancel_task` |
| `GetTaskStatus.srv` | `/get_task_status` |
| `PauseTask.srv` | `/pause_task` |
| `ResumeTask.srv` | `/resume_task` |
| `TriggerEmergencyStop.srv` | `/trigger_estop` |
| `SetInitialPose.srv` | `/set_initial_pose` |

## Messages

| 文件 | 用途 |
|------|------|
| `TaskType.msg` | `IDLE` / `NAVIGATION` / `WAYPOINTS` |
| `TaskState.msg` | 任务生命周期 |
| `TaskStatus.msg` | `/status` |
| `Event.msg` | `/events` |
| `Error.msg` | 统一错误码 |
| `Waypoint.msg` | 多点导航路点 |
| `WaypointStatus.msg` | 各路点结果 |
| `Graph.msg` | 拓扑图 / visibility graph（见下方） |
| `GraphNode.msg` | 图节点（含 start/goal/obstacle 标志） |
| `GraphEdge.msg` | 图边（含权重、可选 path） |
| `GraphFace.msg` | 填充面片（拓扑图专用） |

### Graph / visibility graph

`Graph.kind`:

| 值 | 含义 |
|----|------|
| `KIND_TOPOLOGICAL` | NavMesh 等区域拓扑（可有 `faces`） |
| `KIND_VISIBILITY` | 视线可达图：障碍顶点 + 起终点，边为直线段 |

Visibility graph 发布约定：

- `nodes[].flags`：`FLAG_START` / `FLAG_GOAL` / `FLAG_OBSTACLE_VERTEX`
- `edges[]`：`TYPE_UNDIRECTED`，`LINK_SHAPE_LINE`，`weight`=距离，`path` 留空
- `faces` 留空

Python 辅助：`autonomy_simulator/scripts/graph_visibility.py` 中 `build_visibility_graph()`。

## 构建

```bash
colcon build --packages-select autonomy_msgs
source install/setup.bash
ros2 interface show autonomy_msgs/action/NavigatePose
```
