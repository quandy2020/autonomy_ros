# autonomy_ros

与 `autonomy` **进程隔离**：不链接 `libautonomy`，只通过 **autolink 消息**桥接。

```
[ autonomy 进程 ]  ←──autolink──→  [ autonomy_ros 进程 ]  ←──DDS──→  RViz / 客户端
```

## 依赖

| 链接 | 不链接 |
|------|--------|
| `Autolink`、`automsgs`、`rclcpp` | `libautonomy` |

## 桥接通道（与 autonomy 对齐）

| 方向 | Channel | 用途 |
|------|---------|------|
| autonomy → ROS | `/map` `/plan` `/cmd_vel` `/odom` | 可视化 / 控制输出 |
| autodriver → autonomy | `/odom` `/scan` | 传感器（不经 ROS） |
| ROS → autonomy | `/navigate_to_pose` `/navigate_through_poses` | 导航 Action |

## 外部 ROS API（Nav2 风格）

- Action: `navigate_to_pose` / `navigate_through_poses`
- Service: `set_initial_pose` / `cancel_task`
- Topic: `goal_pose` / `waypoints`

## 启动

```bash
# 1) 先启动 autonomy（mainboard / 导航进程）
# 2) 再启动 ROS 桥
ros2 launch autonomy_ros autonomy.launch.py use_rviz:=true
```
