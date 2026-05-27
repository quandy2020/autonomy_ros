# 定位与 map 坐标系

`autonomy_ros` **不内置** AMCL 或 `robot_localization`。需要全局地图导航时，请自行发布 **`map` → `odom`** TF，并将参数 **`autonomy.planner.global_frame`** 设为与规划一致的坐标系。

---

## 仿真（Gazebo，无 AMCL）

- 默认配置见 `config/parameters.yaml`：`autonomy.planner.global_frame: odom`
- 规划、代价地图与可视化均在 **odom** 下工作，无需 `map` 帧

---

## 真机 / 需要 map 导航

1. 启动定位（Nav2 AMCL、`robot_localization` 等）
2. 修改 `config/parameters.yaml`：
   - `autonomy.planner.global_frame: map`
   - 确保 `map` → `odom` 由定位节点发布
   - `odom` → `base_footprint` 由里程计或仿真发布
3. RViz **2D Pose Estimate** 发布到 `initialpose`（参数 `navigation.init_pose_topic`）；`NavigationServer` 可转发至定位栈订阅的话题

示例（需自行安装 Nav2）：

```bash
# ros2 launch nav2_bringup localization_launch.py map:=/path/to/map.yaml
```

---

## Fake 机器人模式

使用 `simulation_mode:=fake` 时无激光，建议：

```yaml
autonomy:
  enable_scan_bridge: false
  planner:
    global_frame: odom
```

见 [仓库 README](../../README.md) 中 Fake 模式说明。
