# autonomy_ros 文档

| 文档 | 说明 |
|------|------|
| [external_commands.md](external_commands.md) | **外部指令 API**：Action / Service / 配套 Topic + **ROS2 命令行测试** |
| [autonomy_msgs/README.md](../../autonomy_msgs/README.md) | 消息类型定义（`.msg` / `.action` / `.srv`） |

接口由 `autonomy_node` 的 `command` 模块提供，需开启参数 `enable_command: true`（且 `enable_planner`、`enable_controller` 为 true）。
