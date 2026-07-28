# autocharge

ROS 2 自动回充包：仿真 + predock 导航 + 红外对接 FSM。

## 节点

| 节点 | 职责 |
|------|------|
| `env_sim` | 仿真：odom、TF、红外、`/dock/charge_connected` |
| `docking_node` | predock → IR 对接 FSM → `/cmd_vel`；对外 Action `dock_to_charger` |
| `docking_tester` | 批量网格测试（可选） |

配置：`config/sim.yaml`（仿真）、`config/docking.yaml`（回充）。

## 构建

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jdbot_interfaces autocharge
source install/setup.bash
```

## 对外 API（推荐）

### Action：`/dock_to_charger`

接口定义：`jdbot_interfaces/action/AutoCharge`

| 段 | 字段 | 说明 |
|---|---|---|
| **Goal** | `charger_pose` | 充电桩粗略位姿（`geometry_msgs/PoseStamped`，通常为 `odom` 系） |
| | `timeout` | 总任务超时（秒）；≤0 时使用参数 `action_default_timeout_s` |
| **Result** | `success` | 是否成功对接并开始充电 |
| | `error_code` | 0 成功；1 超时；2 导航失败；3 视觉失败；4 逼近失败；5 红外/接触失效；6 充电握手失败；7 取消；8 其他 |
| | `message` | 详细描述 |
| | `total_elapsed_seconds` | 总耗时 |
| **Feedback** | `state` | 0 IDLE … 8 ABORTING（见接口文件注释） |
| | `sub_state` | FSM 子阶段（0 表示无） |
| | `progress` | 0.0~1.0 |
| | `status_msg` | 人类可读状态 |
| | `current_robot_pose` | 当前机器人位姿 |
| | `attempt_count` | 重试次数 |
| | `remaining_time` | 预估剩余时间 |

**成功条件**：`charge_connected` 后置 `success=true`；接触点越过桩面且未接通 → `ABORT`（`fail_on_dock_overrun_enable`）。

#### 开始 / 停止 / 取消

| 操作 | 推荐命令 | 说明 |
|---|---|---|
| **开始** | `ros2 action send_goal /dock_to_charger ...` | 发送 Goal 即开始；运行中重复发送会被拒绝 |
| **取消（同终端）** | 在 `send_goal --feedback` 终端按 **Ctrl+C** | Jazzy 标准 CLI 取消方式 |
| **取消（另开终端）** | `ros2 topic pub --once /dock/abort std_msgs/msg/Bool "{data: true}"` | 等效取消；Result `error_code=7` |

Action 运行期间 **`/dock/start`、`/dock/reset` 会被忽略**；请用 **Ctrl+C** 或 **`/dock/abort`**。

> **注意**：ROS 2 Jazzy 的 `ros2 action` **没有** `cancel_goal` 子命令（仅有 `info` / `list` / `send_goal` / `type`）。

**开始示例**：

```bash
ros2 action send_goal /dock_to_charger jdbot_interfaces/action/AutoCharge "{
  charger_pose: {
    header: {frame_id: 'odom'},
    pose: {
      position: {x: 0.5, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
    }
  },
  timeout: 120.0
}" --feedback
```

**取消示例**：

```bash
# 方式 1：在运行 send_goal 的终端按 Ctrl+C

# 方式 2：另开终端发布 abort（推荐脚本/自动化场景）
ros2 topic pub --once /dock/abort std_msgs/msg/Bool "{data: true}"
```

**查看接口 / 状态**：

```bash
ros2 action info /dock_to_charger
ros2 interface show jdbot_interfaces/action/AutoCharge
ros2 action list
```

Action 名称由参数 `dock_to_charger_action` 配置（默认 `dock_to_charger`）。

### 话题 API（调试 / 兼容）

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/dock/start` | `std_msgs/Bool` | 订阅 | `true` 开始；`false` 暂停（仅话题模式）。Action 运行中忽略 |
| `/dock/reset` | `std_msgs/Bool` | 订阅 | `true` 重置 FSM（可选复位位姿）。Action 运行中忽略 |
| `/dock/abort` | `std_msgs/Bool` | 订阅 | `true` 中止；Action 运行中等效 Ctrl+C 取消 |
| `/dock/state` | `std_msgs/String` | 发布 | FSM 状态名（`SEARCH` / `CENTER` / `SUCCESS` 等） |
| `/dock/charge_connected` | `std_msgs/Bool` | 订阅 | 充电接触/握手信号 |
| `/ir/dock` | `jdbot_interfaces/DockIR` | 订阅 | 红外原始数据 |
| `/odom` | `nav_msgs/Odometry` | 订阅 | 里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 速度指令 |
| `/dock/test_metrics` | `std_msgs/String` | 发布 | 测试指标（`stats_enable=true` 时） |
| `/predock_pose` | `geometry_msgs/PoseStamped` | 发布 | predock 目标（可选） |

**话题方式启动**（使用 `docking.yaml` 默认桩位，不指定 `charger_pose`）：

```bash
ros2 topic pub --once /dock/start std_msgs/msg/Bool "{data: true}"
```

## 工作流

### 仿真联调

```bash
# 终端 1：仿真 + 回充
ros2 launch autocharge autocharge.env.launch.py

# 终端 2：设初始位姿后发送 Action 或 /dock/start
# RViz → 2D Pose Estimate，或发布 /initialpose
```

内部流程：

```text
PREDOCK（可选）→ SEARCH → BIAS_* → CENTER → SUCCESS
                              ↑ 仅 charge_connected 触发 SUCCESS
```

### 实机部署

只启动 `docking_node`，接入真实 `/odom`、`/ir/dock`、`/dock/charge_connected`；对外调用 `/dock_to_charger` Action。

```bash
ros2 run autocharge docking_node --ros-args --params-file install/autocharge/share/autocharge/config/docking.yaml
```

## 关键参数（`config/docking.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `dock_x` / `dock_y` / `dock_yaw` | 0.5 / 0.0 / π | 默认桩位（话题模式）；Action 模式由 Goal 覆盖 |
| `dock_to_charger_action` | `dock_to_charger` | Action 服务名 |
| `action_default_timeout_s` | 300.0 | Goal.timeout≤0 时的默认超时 |
| `predock.enable_nav` | false | 是否先导航到 predock |
| `predock.distance_m` | 0.5 | predock 距桩面距离 |
| `fail_on_dock_overrun_enable` | true | 越界未接通判 ABORT |
| `fail_on_dock_overrun_margin_m` | 0.0 | 越界容差（米） |

完整参数见 `config/docking.yaml`、`config/sim.yaml`。

## 批量测试（可选）

```bash
ros2 launch autocharge autocharge.env.launch.py use_rviz:=false

# 桩前 50 cm 横向矩阵（120 轮）
ros2 launch autocharge autocharge.tester.launch.py test_scenario:=far_50cm

# 桩前 70 cm 横向矩阵（120 轮）
ros2 launch autocharge autocharge.tester.launch.py test_scenario:=far_70cm
```

结果目录：`~/dock_test_logs/run_<timestamp>/`（含 `summary.csv`、`success_rate_by_offset.csv`）。

## 调参

1. 仿真桩位、红外布局 → `config/sim.yaml`（`station_x/y/yaw`）
2. 回充策略与对外接口 → `config/docking.yaml`
3. `colcon build --packages-select autocharge && source install/setup.bash`
4. 重启 launch

## 更多说明

策略与红外布局见 `doc/`。
