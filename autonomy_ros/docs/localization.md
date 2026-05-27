# 定位与 map 坐标系

`autonomy_ros` **不内置** AMCL / `robot_localization`；你需要自行发布 **`map` → `odom`**（以及传感器 TF），并与参数 **`autonomy.planner.global_frame`** 一致。

## 仿真（Gazebo，无 AMCL）

- `config/autonomy_params.yaml` 默认：`autonomy.planner.global_frame: odom`
- 规划、跟随、代价地图均在 **odom** 下工作；无需 `map` 帧。

## 真机 / 需要全局地图导航

1. 启动定位节点（任选其一）：
   - Nav2 `amcl`
   - `robot_localization` EKF + 地图匹配
2. 设置 `autonomy.planner.global_frame: map`，并保证：
   - `map` → `odom` 由定位发布
   - `odom` → `base_footprint` 由里程计 / 仿真发布
3. RViz **2D Pose Estimate** → `CommandInterface` 转发到 `/initialpose`（若你的定位栈订阅该话题）。

可选 launch 模板（仅文档占位，不安装第三方包）：

```bash
# 在你自己的 workspace 中 include AMCL，例如：
# ros2 launch nav2_bringup localization_launch.py map:=/path/to/map.yaml
```

## Follow `target_id`

感知需发布 `vision_msgs/Detection3DArray`（参数 `autonomy.follow_detections_topic`）。检测的 `id` 或 `results[].hypothesis.class_id` 与 Follow goal 的 `target_id` 匹配；位姿经 TF 变换到 `autonomy.planner.global_frame`。
