# 导航触发（经 autolink）

ROS 侧收到目标后，转发到 autonomy 进程的 autolink Action：

| ROS | Autolink |
|-----|----------|
| `/navigate_to_pose` | `/navigate_to_pose` |
| `/navigate_through_poses` | `/navigate_through_poses` |
| `/goal_pose` | 同上（便捷 Topic） |
| `/waypoints` | 同上（`Path` → 多点） |

需先启动 autonomy 进程，再启动 `autonomy_ros`。
