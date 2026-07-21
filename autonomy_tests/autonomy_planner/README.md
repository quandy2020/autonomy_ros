# autonomy_planner

ROS 2 包：`autonomy::planning` 的 gtest，以及基于 InteractiveMarker 障碍物点云的全局规划仿真。

## 仿真架构

```
config/data/*.yaml|.pgm
        │ load
        ▼
   /map (OccupancyGrid) ──► RViz Map
        │
        ▼
StaticLayer + ObstacleLayer + Inflation
        │
InteractiveMarker ──► PointCloud2 ──► Costmap2DWrapper ──► NavFn/Dijkstra/Theta*
        │                                    │
        │                                    ▼
fake_robot ◄── odom ◄── /initialpose (set_pose)   /planner_sim/plan + global_costmap
```

- **静态地图**：默认 `share/autonomy/config/data/map.yaml`（可用 `map_file:=...` 切换）
- **交互流程**：RViz **2D Pose Estimate** → `/initialpose` 设起点并移动机器人；**2D Nav Goal** → `/goal_pose` 设目标；**两者都收到后**触发一次规划，完成后需重新设置起点与目标
- **静态障碍**：RViz `Interact` 直接拖拽圆柱本体，在 XY 平面移动
- **动态障碍**：绕圆心轨道运动（亦可拖拽改轨道中心）
- **规划器**：直连插件 `CreatePlan`（不依赖 PlannerServer autolink action）

## 启动

```bash
colcon build --packages-select autonomy autonomy_ros autonomy_simulator autonomy_planner --symlink-install
source install/setup.bash

# 默认 NavFn + config/data/map.yaml
ros2 launch autonomy_planner planner_sim.launch.py

# 切换规划器 / 地图
ros2 launch autonomy_planner planner_sim.launch.py planner_id:=dijkstra_planner
ros2 launch autonomy_planner planner_sim.launch.py map_file:=turtlebot3_house.yaml
ros2 launch autonomy_planner planner_sim.launch.py map_file:=Shanghai_0_1024.yaml

# 障碍数量
ros2 launch autonomy_planner planner_sim.launch.py \
  planner_id:=navfn_planner static_count:=4 dynamic_count:=1
```

`planner_id`：`navfn_planner` | `dijkstra_planner` | `theta_star_planner`

## 参数配置

| 文件 | 说明 |
|------|------|
| `config/planner_sim.yaml` | 仿真节点：帧、planner_id、map_file、costmap |
| `config/obstacle_marker.yaml` | InteractiveMarker 障碍点云 |

算法参数来自 `autonomy` 的 `config/planner/planner.lua`（可通过 `configuration_directory` 覆盖）。

## RViz

- **Map** (`/map`) 与 **GlobalCostmap**：均在 `map` 坐标系，RViz Fixed Frame 设为 `map`
- **Interact**：拖静态/动态圆柱 → 点云与代价地图更新
- **SetInitialPose**：发 `/initialpose` → 设置机器人起点（TF 同步更新）
- **SetGoal**：发 `/goal_pose` → 与起点齐备后规划并发布 `/planner_sim/plan`

`map.yaml` 原点在 `(0,0)`、范围约 `[0,20]×[0,20]`，请在地图范围内设目标。

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
