# autonomy_autocharge

Habitat + Nav2 + `autocharge` 联调 demo：**predock 导航 → IR 回充 → 离桩回 predock → 导航原点**。

```
spawn → Nav2 predock → dock_to_charger → 充电保持 → Nav2 predock → Nav2 origin
```

- predock / 返程：**Nav2**（`docking_node` 中 `predock.enable_nav=false`）
- 对接段：**autocharge** FSM → `/cmd_vel`
- IR / 充电仿真：**station_ir_sim**

## 构建与启动

```bash
source /opt/ros/humble/setup.bash   # 或 jazzy
colcon build --packages-select jdbot_interfaces autocharge autonomy_autocharge \
  autonomy_ros autonomy_simulator --symlink-install
source install/setup.bash

ros2 launch autonomy_autocharge autocharge_demo.launch.py
# 或：bash scripts/autocharge_demo.sh
```

## 默认位姿（map 系）

| 对象 | x | y | yaw |
|------|---|---|-----|
| spawn | -9.0 | 0.0 | 0 |
| 充电桩 | 2.2 | 0.0 | π |
| predock | 1.4 | -0.08 | 0 |
| 返程原点 | 0.0 | 0.0 | 0 |

桩/predock 须在 `/map` 可通行区域。位姿默认值在 **`config/autocharge_demo.yaml`**；launch 的 `charger_*` / `predock_*` 默认为空，**改 yaml 即可**，命令行传入时才覆盖。

## 常用参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `run_return_mission` | `true` | `false` 对接成功即停 |
| `post_dock_hold_s` | `3.0` | 充电停留（s） |
| `demo_start_delay` | `25.0` | 任务开始延迟（s） |
| `spawn_x/y/yaw` | `-9/0/0` | Habitat 出生 |
| `origin_x/y/yaw` | `0/0/0` | 返程终点 |
| `use_rviz` | `true` | `rviz/autocharge_demo.rviz`（无 SetGoal） |

```bash
# 仅回充
ros2 launch autonomy_autocharge autocharge_demo.launch.py run_return_mission:=false

# 命令行覆盖桩位
ros2 launch autonomy_autocharge autocharge_demo.launch.py charger_x:=2.2
```

## 配置

| 文件 | 用途 |
|------|------|
| `config/autocharge_demo.yaml` | 桩位、demo、IR 仿真 |
| `config/docking_demo.yaml` | 对接 FSM |
| `config/nav2_autocharge_demo_params.yaml` | Nav2 MPPI |

对接细节见 [`autocharge` README](../../autonomy_planning/autocharge/README.md)。

## 调试

```bash
ros2 topic echo /dock/state
ros2 param get /charger_markers_node charger_x
ros2 topic pub --once /dock/abort std_msgs/msg/Bool "{data: true}"
```

- Nav2 不动：检查 costmap `inflation_radius` ≥ `robot_radius`（0.15 m）
- **勿用 RViz「2D Goal Pose」**，会抢占 demo 导航目标
- Docker 内需额外 `source install/setup.bash`
