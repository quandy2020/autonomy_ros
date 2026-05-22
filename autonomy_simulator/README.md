# autonomy_simulator

TurtleBot3 仿真资源包：Gazebo（Ignition）与 **无 Gazebo 的 fake 差速小车**（参考 [ROBOTIS turtlebot3_fake_node](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_fake_node)）。

## 启动方式

### Gazebo 仿真（默认）

```bash
ros2 launch autonomy_simulator tb3_simulator.launch.py
ros2 launch autonomy_simulator tb3_simulator.launch.py headless:=True
```

### Fake 机器人（无需 Gazebo）

订阅 `geometry_msgs/TwistStamped` 的 `cmd_vel`，发布 `odom`、`joint_states`、`tf`（与 `autonomy_ros` 一致）：

```bash
ros2 launch autonomy_simulator fake_robot.launch.py
```

### 统一入口（`sim_mode`）

```bash
# Gazebo
ros2 launch autonomy_simulator simulator.launch.py sim_mode:=gazebo

# Fake
ros2 launch autonomy_simulator simulator.launch.py sim_mode:=fake use_sim_time:=false
```

### 完整栈（`autonomy_ros`）

```bash
# Gazebo + 导航栈 + RViz
ros2 launch autonomy_ros autonomy_stack.launch.py

# Fake 机器人 + 导航栈（建议关闭仿真时钟与激光桥）
ros2 launch autonomy_ros autonomy_stack.launch.py \
  sim_mode:=fake use_sim_time:=false
```

Fake 模式下请在 `autonomy_params.yaml` 中将 `autonomy.enable_scan_bridge` 设为 `false`（无 `/scan`），`autonomy.planner.global_frame` 使用 `odom`。

## 目录

| 路径 | 说明 |
|------|------|
| `src/fake_robot_node.cpp` | 差速积分 fake 节点 |
| `launch/` | `tb3_simulator`、`fake_robot`、`simulator` |
| `param/fake_robot_waffle.yaml` | Waffle 轮距/轮径等参数 |
| `urdf/` / `worlds/` / `models/` / `configs/` | Gazebo 资源 |

## 节点

| 可执行文件 | 说明 |
|------------|------|
| `fake_robot_node` | `cmd_vel` → 里程计 + TF + 关节状态 |
