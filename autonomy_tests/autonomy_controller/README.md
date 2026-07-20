# autonomy_controller

ROS 2 包：`autonomy::control` 的 gtest，以及基于 InteractiveMarker 障碍物点云的控制器仿真。

## 仿真架构

```
InteractiveMarker (静/动态圆柱)
        │ sample
        ▼
PointCloud2  ──► Costmap2DWrapper ──► MPPI / RPP / Graceful
        │                                    │
        │                                    ▼
fake_robot ◄────────────────────────── cmd_vel (TwistStamped)
   odom ──────────────────────────────────┘
```

- **静态障碍**：RViz `Interact` 直接拖拽圆柱本体，在 XY 平面移动
- **动态障碍**：绕圆心轨道运动（亦可用 RViz `Interact` 拖拽圆柱，修改轨道中心）
- **控制器**：直连插件 `Configure` / `Activate` / `SetPlan` / `ComputeVelocityCommands`（不依赖 ControllerServer）

## 启动

```bash
colcon build --packages-select autonomy autonomy_ros autonomy_simulator autonomy_controller --symlink-install
source install/setup.bash

# 默认 MPPI + 圆路径
ros2 launch autonomy_controller controller_sim.launch.py

# 切换控制器
ros2 launch autonomy_controller controller_sim.launch.py controller_id:=rpp
ros2 launch autonomy_controller controller_sim.launch.py controller_id:=graceful

# 8 字路径 / 关 RViz / 障碍数量
ros2 launch autonomy_controller controller_sim.launch.py \
  path_shape:=figure_eight controller_id:=mppi \
  static_count:=4 dynamic_count:=1
```

`controller_id`：`mppi` | `rpp` | `graceful`

## 参数配置

默认参数在 `config/`（launch 自动加载，命令行可覆盖）：

| 文件 | 说明 |
|------|------|
| `config/controller_sim.yaml` | 仿真节点：帧、路径、goal checker、costmap、MPPI 可视化 |
| `config/obstacle_marker.yaml` | InteractiveMarker 障碍点云 |
| `config/presets/goal_navigation.yaml` | RViz 2D Goal 导航（`repeat_path: false`） |
| `config/presets/mppi_performance.yaml` | 降低 viz / costmap 频率 |

MPPI/RPP/Graceful **算法参数** 仍来自 `autonomy` 的 `config/control/controller.lua`（可通过 `configuration_directory` 覆盖）。

```bash
# Goal 导航预设
ros2 launch autonomy_controller controller_sim.launch.py \
  config_preset:=goal_navigation controller_id:=mppi

# 性能预设 + 自定义障碍数
ros2 launch autonomy_controller controller_sim.launch.py \
  config_preset:=mppi_performance static_count:=2
```

## RViz

- **Interact**：直接拖 **红色/蓝色圆柱**（InteractiveMarker），不是橙色点云
- 默认关闭 `ObstacleCloud` 显示，避免点云挡住圆柱导致点不中；障碍仍进 costmap（看 `LocalCostmap`）
- Displays：参考路径 / 执行路径 / 局部代价地图 / InteractiveMarkers
- **MPPI**（`controller_id:=mppi`）：
  - `/controller_sim/mppi_candidates`：青色半透明候选轨迹（MarkerArray）
  - `/controller_sim/mppi_optimal_path`：橙色最优轨迹（Path + Marker）

## gtest

```bash
colcon test --packages-select autonomy_controller --event-handlers console_direct+
colcon test-result --verbose
```

| 测试 | 覆盖 |
|------|------|
| `test_constants` … `test_control_options` | constants / conversions / utils / checkers / SmoothControlLaw / smoother / motion models / exceptions / `controller.lua` |
