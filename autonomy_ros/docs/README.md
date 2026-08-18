# autonomy_ros 文档索引

本目录描述 **autonomy_ros** 包的架构、对外接口与使用方式。消息定义见同级包 **autonomy_msgs**。

## 文档列表

| 文档 | 说明 |
|------|------|
| [architecture.md](architecture.md) | 源码目录、模块职责、数据流 |
| [external_commands.md](external_commands.md) | Action / Service / Topic 字段与 `ros2` 示例 |
| [navigation_client.md](navigation_client.md) | `navigation_client.py` 命令行客户端 |
| [localization.md](localization.md) | `map` / `odom` 坐标系与定位接入 |
| [conversions.md](conversions.md) | `autonomy_ros::{fromRos,toRos}` ROS ↔ automsgs |

## 快速入口

```bash
# 启动仿真 + 导航栈
ros2 launch autonomy_ros navigation_stack.launch.py use_rviz:=true

# 导航 CLI
ros2 run autonomy_ros navigation_client.py list-tasks
ros2 run autonomy_ros navigation_client.py run-task go_to_point_a --feedback

# 查看任务状态
ros2 topic echo /status
```

## 相关包

- [autonomy_msgs README](../../autonomy_msgs/README.md) — `NavigatePose` / `NavigateThrough` 等消息定义
- [仓库 README](../../README.md) — 编译、启动与工程结构
