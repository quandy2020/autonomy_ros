# autonomy_ros 文档

| 文档 | 说明 |
|------|------|
| [autonomy_cmd.md](autonomy_cmd.md) | **`autonomy_cmd.py` 使用指南**（任务配置、子命令、示例） |
| [architecture.md](architecture.md) | **架构图**：`Autonomy` 门面、`system::AutonomyNode`、ROS Bridge、数据流 |
| [conversions.md](conversions.md) | **ROS ↔ commsgs**：`fromRos` / `toRos` 转换模块约定 |
| [localization.md](localization.md) | **map / odom** 与 AMCL；`autonomy.planner.global_frame` 选型 |
| [external_commands.md](external_commands.md) | **外部指令 API**：Action / Service / 配套 Topic + **ROS2 命令行测试** |
| [autonomy_msgs/README.md](../../autonomy_msgs/README.md) | 消息类型定义（`.msg` / `.action` / `.srv`） |

对外 Action/Service 由 `CommandInterface` 提供，需 `enable_command: true` 且 `enable_autonomy: true`（核心栈由 `autonomy_ros::Autonomy` 启动）。

命令行工具 **`autonomy_cmd.py`** 的完整说明见 **[autonomy_cmd.md](autonomy_cmd.md)**（含 `tasks.json` 任务配置、重复导航、mission 序列与全部子命令示例）。
