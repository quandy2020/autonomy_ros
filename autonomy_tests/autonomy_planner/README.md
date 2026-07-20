# autonomy_planner

ROS 2 包：`autonomy::planning` 的 gtest，以及基于 InteractiveMarker 障碍物点云的全局规划仿真。

## 仿真架构

```
InteractiveMarker (静/动态圆柱)
        │ sample
        ▼
PointCloud2  ──► Costmap2DWrapper ──► NavFn / Dijkstra / Theta*
        │                                    │
        │                                    ▼
fake_robot ◄── odom                  nav_msgs/Path (planner_sim/plan)
```

- **静态障碍**：RViz `Interact` 直接拖拽圆柱本体，在 XY 平面移动
- **动态障碍**：绕圆心轨道运动（亦可拖拽改轨道中心）
- **规划器**：直连插件 `CreatePlan`（不依赖 PlannerServer autolink action）

## 启动

```bash
colcon build --packages-select autonomy autonomy_ros autonomy_simulator autonomy_planner --symlink-install
source install/setup.bash

# 默认 NavFn
ros2 launch autonomy_planner planner_sim.launch.py

# 切换规划器
ros2 launch autonomy_planner planner_sim.launch.py planner_id:=dijkstra_planner
ros2 launch autonomy_planner planner_sim.launch.py planner_id:=theta_star_planner

# 障碍数量
ros2 launch autonomy_planner planner_sim.launch.py \
  planner_id:=navfn_planner static_count:=4 dynamic_count:=1
```

`planner_id`：`navfn_planner` | `dijkstra_planner` | `theta_star_planner`

## 参数配置

| 文件 | 说明 |
|------|------|
| `config/planner_sim.yaml` | 仿真节点：帧、planner_id、costmap |
| `config/obstacle_marker.yaml` | InteractiveMarker 障碍点云 |

算法参数来自 `autonomy` 的 `config/planner/planner.lua`（可通过 `configuration_directory` 覆盖）。

## RViz

- **Interact**：直接拖静态/动态圆柱本体 → 点云与全局代价地图更新
- **SetGoal**：发 `/goal_pose` → 从当前 odom 位姿规划到目标，发布 `/planner_sim/plan`
- Displays：全局路径 / 全局代价地图 / 障碍点云 / InteractiveMarkers

## gtest

```bash
colcon test --packages-select autonomy_planner --event-handlers console_direct+
colcon test-result --verbose
```

| 测试 | 覆盖 |
|------|------|
| `test_planner_options` | `planner.lua` / `CreateOptions` |
| `test_navfn_planner` | NavFn 空旷路径 / 取消 |
| `test_dijkstra_planner` | Dijkstra 空旷路径 |
| `test_theta_star_planner` | Theta* 空旷路径 |
| `test_path_simplifier` | Douglas-Peucker 简化冒烟 |
