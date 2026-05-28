# navigation_client.py

`navigation_client.py` 是导航命令行客户端，调用 `NavigationService` 暴露的 Action 与 Service。

---

## 前置条件

```bash
source install/setup.bash
ros2 launch autonomy_ros navigation_stack.launch.py
ros2 action list | grep navigate
```

---

## 子命令

| 子命令 | 说明 |
|--------|------|
| `list-tasks` | 列出 `navigation_tasks.json` 中的位姿与命名任务 |
| `run-task <name>` | 执行命名任务 |
| `navigate-pose` | 发送单点导航（`--x --y --yaw --frame`） |
| `navigate-through` | 从 JSON 文件发送多点导航（`--goal-file`） |
| `cancel` | 取消任务（`--all` 取消全部） |
| `status` | 查询任务状态 |
| `pause` / `resume` | 暂停 / 恢复 |
| `estop` | 急停（`--release` 解除） |
| `set-initial-pose` | 重定位 |

通用选项：`--namespace`、`--feedback`、`--timeout`。

---

## 示例

```bash
# 列出任务配置
ros2 run autonomy_ros navigation_client.py list-tasks

# 运行 config/navigation_tasks.json 中的命名任务
ros2 run autonomy_ros navigation_client.py run-task go_to_point_a --feedback

# 直接指定坐标
ros2 run autonomy_ros navigation_client.py navigate-pose --x 1.0 --y 0.5 --feedback

# 多点导航
PKG=$(ros2 pkg prefix autonomy_ros)
ros2 run autonomy_ros navigation_client.py navigate-through \
  --goal-file $PKG/share/autonomy_ros/config/navigate_through_goal.json

# 任务控制
ros2 run autonomy_ros navigation_client.py pause --reason manual
ros2 run autonomy_ros navigation_client.py cancel --all
```

---

## 任务配置文件

默认路径：安装后的 `share/autonomy_ros/config/navigation_tasks.json`。

环境变量 **`AUTONOMY_NAVIGATION_TASKS`** 可指定其他 JSON 路径。

示例结构：

```json
{
  "frame": "odom",
  "poses": {
    "point_a": { "x": 1.0, "y": 0.5, "yaw": 0.0 }
  },
  "tasks": {
    "go_to_point_a": { "type": "navigate_pose", "pose": "point_a" },
    "patrol_home": {
      "type": "navigate_through",
      "poses": ["point_a", "point_b", "home"]
    }
  }
}
```

---

## 相关文档

- [external_commands.md](external_commands.md) — Action / Service 字段说明
- [architecture.md](architecture.md) — `NavigationService` 在栈中的位置
