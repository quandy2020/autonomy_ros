# autonomy_simulator

TurtleBot3 仿真资源包：Gazebo（Ignition）、**fake 差速小车**（参考 [ROBOTIS turtlebot3_fake_node](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_fake_node)），以及 **Habitat-Sim 桥接**（Python）。

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

# Habitat（需预先安装 habitat-sim 与 MP3D 场景）
ros2 launch autonomy_simulator simulator.launch.py sim_mode:=habitat
```

### Habitat 桥接（Python，MP3D）

加载 MP3D 场景（默认 `17DRP5sb8fy`），订阅 `cmd_vel`（`geometry_msgs/Twist`，与 Nav2 一致），发布：

- `odom`、`joint_states`（`robot_state_publisher` + `urdf/habitat.urdf` 生成 TF）
- `habitat/agent_pose`、`habitat/set_agent_pose`（位姿读写）
- `camera/rgb/image_raw`、`camera/depth/image_raw`、`camera/semantic/image_raw`
- `semantic_pointcloud`（PLY 点云）、`map`（占据栅格）

```bash
ros2 launch autonomy_simulator habitat.launch.py

# 指定场景数据目录
ros2 launch autonomy_simulator habitat.launch.py \
  scene_data_path:=/workspace/autonomy/src/17DRP5sb8fy
```

代码位于 `scripts/habitat/`（Google Python 风格 + ROS 2 节点规范）：

| 模块 | 职责 |
|------|------|
| `habitat_node.py` | 可执行入口（`rclpy.init` / `spin` / `shutdown`） |
| `node.py` | `BridgeNode`（订阅、定时器、会话编排） |
| `config.py` | `Config` 数据类与 `load()` 参数加载 |
| `sim.py` | `Session`（Habitat-Sim 场景、差速步进、传感器） |
| `odom.py` | `OdomPublisher`（`/joint_states` + `/odom`） |
| `camera.py` | `CameraPublisher`（rgb / depth / semantic 图像） |
| `ply.py` | `PlyPublisher`（语义点云 + 占据栅格） |
| `coords.py` | Habitat ↔ ROS 坐标转换 |

参数见 `param/habitat.yaml`。

### 完整栈（`autonomy_ros`）

```bash
# Gazebo + 导航栈 + RViz
ros2 launch autonomy_ros navigation_stack.launch.py

# Fake 机器人 + 导航栈（建议关闭仿真时钟与激光桥）
ros2 launch autonomy_ros navigation_stack.launch.py \
  simulation_mode:=fake use_sim_time:=false

# Habitat + 导航栈
ros2 launch autonomy_ros navigation_stack.launch.py \
  simulation_mode:=habitat use_sim_time:=true
```

Fake 模式下请在 `config/parameters.yaml` 中将 `autonomy.enable_scan_bridge` 设为 `false`（无 `/scan`），`autonomy.planner.global_frame` 使用 `odom`。

## 目录

| 路径 | 说明 |
|------|------|
| `src/fake_robot_node.cpp` | 差速积分 fake 节点（C++） |
| `scripts/habitat_node.py` | Habitat 桥接入口 |
| `scripts/habitat/` | Habitat 桥接 Python 包 |
| `launch/` | `tb3_simulator`、`fake_robot`、`habitat`、`simulator` |
| `param/fake_robot_waffle.yaml` | Waffle 轮距/轮径等参数 |
| `param/habitat.yaml` | Habitat 桥接参数 |
| `urdf/` / `worlds/` / `models/` / `configs/` | Gazebo 与 Habitat 资源 |

## 节点

| 可执行文件 | 说明 |
|------------|------|
| `fake_robot_node` | `cmd_vel` → 里程计 + TF + 关节状态（C++） |
| `habitat_node.py` | Habitat 仿真 ↔ ROS 话题桥接（Python） |
