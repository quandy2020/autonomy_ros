# autonomy_simulator

ROS 2 仿真资源包：**Gazebo（TurtleBot3）**、**fake 差速小车**、**Habitat-Sim MP3D 桥接**。

## 快速启动

| 模式 | 命令 |
|------|------|
| Gazebo | `ros2 launch autonomy_simulator tb3_simulator.launch.py` |
| Fake | `ros2 launch autonomy_simulator fake_robot.launch.py` |
| Habitat | `ros2 launch autonomy_simulator simulator.launch.py sim_mode:=habitat` |

统一入口：`simulator.launch.py`，参数 `sim_mode:=gazebo \| fake \| habitat`。

> Fake / Habitat 使用系统时钟，请设 `use_sim_time:=false`。

## Habitat 桥接

加载 MP3D 场景（默认 `17DRP5sb8fy`），差速 `cmd_vel` 驱动机器人，发布相机、里程计、语义点云与占据栅格。

**主要话题**

| 话题 | 说明 |
|------|------|
| `cmd_vel` | 速度控制（Nav2 兼容） |
| `odom` / `tf` | 里程计与 `map→odom→base_footprint` |
| `camera/rgb|depth|semantic/image_raw` | 车载 RGB-D-语义 |
| `camera/topdown/image_raw` | 俯视 RGB（可选） |
| `semantic_pointcloud` / `map` | 语义点云 / 占据栅格 |
| `habitat/graph` | NavMesh 拓扑 |

**推荐启动（RViz + 行人 + 交互俯视）**

```bash
ros2 launch autonomy_simulator simulator.launch.py \
  sim_mode:=habitat \
  use_rviz:=true \
  topdown_enabled:=true \
  topdown_mode:=interactive \
  pedestrians_enabled:=true \
  pedestrian_count:=5
```

**俯视相机 `topdown_mode`**

| 值 | 行为 |
|----|------|
| `room` | 正交全览，固定房间中心 |
| `oblique` / `overhead` | 跟随机器人 |
| `interactive` | RViz 6-DOF 标记拖动视角，滚轮菜单调 FOV |

RViz 3D 主视图：左键旋转、滚轮缩放（`rviz/habitat.rviz`）。Image 面板仅 2D 缩放。

## 动态行人（Humanoid）

NavMesh 上游走 + 互避，Habitat 内 **URDF 人形 mesh** 渲染（TrackVLA 资源）。

- 数据路径：`autonomy_lerobot/config/data_paths.yaml` → `humanoid_data_root`
- 每人自动分配不同 avatar（扫描数据目录；可用 `humanoid_avatars` 限定列表）
- RViz：`TrackedPersons`、`PedestrianVisualization`

```bash
# 仅行人，无 RViz
ros2 launch autonomy_simulator simulator.launch.py \
  sim_mode:=habitat pedestrians_enabled:=true pedestrian_count:=5
```

参数见 `param/habitat.yaml`（`pedestrians_*`、`humanoid_*`、`topdown_*`）。

## 与 autonomy_ros 导航栈

```bash
ros2 launch autonomy_ros navigation_stack.launch.py \
  simulation_mode:=habitat use_sim_time:=false
```

Fake 模式请将 `autonomy.enable_scan_bridge` 设为 `false`（无激光 `/scan`）。

## 目录

```
launch/          tb3_simulator, fake_robot, habitat, simulator
param/           habitat.yaml, fake_robot_waffle.yaml
scripts/habitat/ 桥接 Python 包（sim, camera, pedestrians, humanoid, …）
scripts/habitat_node.py
src/             fake_robot_node（C++）
urdf/ worlds/ models/ configs/ rviz/
```

## 可执行节点

| 节点 | 说明 |
|------|------|
| `fake_robot_node` | `cmd_vel` → odom + TF |
| `habitat_node.py` | Habitat-Sim ↔ ROS 话题 |
