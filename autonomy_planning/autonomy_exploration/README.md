# autonomy_exploration

ROS 2 C++ 包装：调用 `autonomy::exploration`，在 **Habitat** 仿真中用 **RViz2** 验证。

## 依赖

- `autonomy`（核心库，含 exploration）
- `autonomy_ros`（ROS ↔ automsgs 转换）
- `autonomy_simulator`（Habitat 桥）
- ROS 2：`rclcpp`、`tf2_ros`、`rviz2` 等

先编译并 source：

```bash
colcon build --packages-up-to autonomy_exploration --symlink-install
source install/setup.bash
```

## 启动（Habitat + 探索 + RViz）

```bash
ros2 launch autonomy_exploration exploration_habitat.launch.py
```

常用参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_rviz` | true | 启动 exploration.rviz |
| `enable_cmd_vel` | true | 简单 P 控制跟踪探索 waypoint |
| `topdown_enabled` | false | Habitat 俯视相机（额外渲染路，默认关） |
| `use_sim_time` | false | Habitat 用墙上时钟 |

## 话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `odom` | in | Habitat 里程计 |
| `camera/depth/image_raw` | in | 深度 |
| `camera/rgb/camera_info` | in | 内参（与 depth 共用光学坐标） |
| `cmd_vel` | out | 可选跟踪速度 |
| `exploration/path` | out | 探索路径 |
| `exploration/waypoint` | out | 当前 lookahead |
| `exploration/costmap` | out | 局部 occupancy |
| `exploration/markers` | out | 路径/航点 Marker |
| `exploration/progress` | out | 进度 [0,1] |

## 仅节点（仿真已起）

```bash
ros2 run autonomy_exploration exploration_node --ros-args \
  --params-file $(ros2 pkg prefix autonomy_exploration)/share/autonomy_exploration/config/exploration_params.yaml
```
