# autonomy_cmd.py 使用指南

`autonomy_cmd.py` 是 `autonomy_ros` 的命令行客户端，对应 `CommandInterface` 注册的 **`/autonomy/*` Action 与 Service**。支持两种用法：

1. **直接子命令**：运行时传坐标、参数（适合单次调试）。
2. **任务配置 `tasks.json`**：命名位姿、可复用任务、**同一 pose 重复导航**、mission 序列（适合展厅/巡检脚本）。

消息字段说明见 [external_commands.md](external_commands.md)；栈架构见 [architecture.md](architecture.md)。

### 单文件脚本

所有逻辑在 **`scripts/autonomy_cmd.py`** 中，自上而下分为：

1. `TaskCatalog` — 加载 `tasks.json`，展开 `repeat` / mission  
2. Goal 构建函数 — 从坐标、JSON、配置生成 Action Goal  
3. `AutonomyCmd` / `TaskRunner` — ROS 调用与任务步骤执行  
4. CLI 子命令注册表  

流程：`子命令` → Goal 构建（或 `TaskCatalog` 展开）→ `AutonomyCmd.run_action` / `call_service`。

---

## 前置条件

| 项 | 要求 |
|----|------|
| ROS 2 | 已 `source` 安装/编译后的 workspace |
| 节点 | `autonomy_node` 已启动，且 `enable_autonomy:=true`、`enable_command:=true` |
| 仿真示例 | `ros2 launch autonomy_ros autonomy_stack.launch.py` |
| 坐标系 | 默认 `odom`（Gazebo 无 AMCL）；有定位时用 `map`，见 [localization.md](localization.md) |

检查节点与接口：

```bash
ros2 node list | grep autonomy
ros2 action list | grep autonomy
ros2 service list | grep autonomy
```

---

## 调用方式

```bash
# 推荐：安装后通过 ros2 run
ros2 run autonomy_ros autonomy_cmd.py <子命令> [参数...]

# 开发时可直接运行（需在 scripts 目录或已安装到 lib/autonomy_ros）
python3 src/autonomy_ros/autonomy_ros/scripts/autonomy_cmd.py <子命令> ...

# 查看全部子命令
ros2 run autonomy_ros autonomy_cmd.py --help
ros2 run autonomy_ros autonomy_cmd.py navigate-pose --help
```

**环境变量（可选）**

| 变量 | 作用 |
|------|------|
| `AUTONOMY_TASKS_CONFIG` | 默认 `tasks.json` 路径，可省略每次 `--config` |

```bash
export AUTONOMY_TASKS_CONFIG=$HOME/my_robot/tasks.json
```

**安装后示例文件路径**

```bash
PKG=$(ros2 pkg prefix autonomy_ros)
echo $PKG/share/autonomy_ros/scripts/examples/tasks.json
```

---

## 通用参数

多数 Action / 任务类子命令支持：

| 参数 | 说明 |
|------|------|
| `--namespace` | 接口前缀，默认 `/autonomy/`；传 `autonomy` 等价于默认，传 `robot` 则为 `/robot/` |
| `--task-id` | 任务 ID，空则自动生成 |
| `--feedback` | 打印 Action 反馈 |
| `--timeout` | 等待结果超时（秒），`0` 表示最长约 3600s |
| `--frame` | 位姿坐标系，默认 `odom` |
| `--x` `--y` `--z` `--yaw` | 位姿（`yaw` 为弧度） |

---

## 一、任务配置模式（推荐）

配置文件为 JSON，默认示例：

`share/autonomy_ros/scripts/examples/tasks.json`

### 1.1 文件结构

```json
{
  "defaults": {
    "frame": "odom",
    "timeout": 120.0,
    "feedback": false,
    "pause_between": 1.0
  },
  "poses": {
    "point_a": { "x": 1.0, "y": 0.5, "z": 0.0, "yaw": 0.0 }
  },
  "tasks": { ... },
  "missions": { ... }
}
```

| 段 | 含义 |
|----|------|
| `defaults` | 全局默认：`frame`、`timeout`、`pause_between` 等 |
| `poses` | **命名位姿**，任务里用字符串引用，避免重复写坐标 |
| `tasks` | 预定义任务，可设 `repeat` 对同一 pose **重复执行导航** |
| `missions` | 按顺序执行多个 task 或内联步骤 |

### 1.2 支持的任务类型（`tasks` / 内联步骤）

| `type` | 说明 | 主要字段 |
|--------|------|----------|
| `navigate_pose` | 单点导航 | `pose`（poses 中的名字）、`repeat`、`pause_between`、`max_speed` |
| `navigate_through` | 多点顺序导航 | `poses`: `["point_a", "point_b", "home"]`、`repeat` |
| `guided_tour` | 展厅导览 | `goal_file`（相对配置文件目录的 JSON） |
| `dock` | 回充 | `dock_id` |
| `wait` | 仅等待 | `seconds` |

`repeat`：对**整条任务**重复 N 次（例如同一 `navigate_pose` 到 `point_a` 跑 3 遍，每次间隔 `pause_between` 秒）。

### 1.3 子命令

#### `list-config` — 查看配置

```bash
ros2 run autonomy_ros autonomy_cmd.py list-config

# 或指定文件
ros2 run autonomy_ros autonomy_cmd.py list-config \
  --config $PKG/share/autonomy_ros/scripts/examples/tasks.json
```

#### `run-pose` — 对命名位姿导航（可重复）

```bash
# 去 point_a 一次
ros2 run autonomy_ros autonomy_cmd.py run-pose point_a

# 同一 pose 连续导航 5 次，每次间隔 2s
ros2 run autonomy_ros autonomy_cmd.py run-pose point_a --repeat 5 --pause-between 2.0 --feedback

# 使用自定义 tasks.json
ros2 run autonomy_ros autonomy_cmd.py run-pose home --repeat 3 --config ~/tasks.json
```

#### `run-task` — 执行预定义任务

```bash
# 示例 tasks.json 中的 goto_a_x3：到 point_a 重复 3 次
ros2 run autonomy_ros autonomy_cmd.py run-task goto_a_x3 --feedback

# 多点巡逻 A -> B -> home
ros2 run autonomy_ros autonomy_cmd.py run-task patrol_ab

# 导览（需同目录下的 guided_tour.json）
ros2 run autonomy_ros autonomy_cmd.py run-task hall_tour
```

#### `run-mission` — 执行任务序列

```bash
# 先 patrol_ab，再 goto_a_x3
ros2 run autonomy_ros autonomy_cmd.py run-mission demo_patrol --feedback

# 某步失败仍继续后续步骤
ros2 run autonomy_ros autonomy_cmd.py run-mission stress_point_a --continue-on-failure
```

### 1.4 自定义 `tasks.json` 示例

复制示例后修改：

```bash
cp $PKG/share/autonomy_ros/scripts/examples/tasks.json ~/my_tasks.json
export AUTONOMY_TASKS_CONFIG=~/my_tasks.json
```

新增命名位姿与重复巡逻任务：

```json
{
  "poses": {
    "dock_front": { "x": 0.5, "y": 0.0, "yaw": 3.14 }
  },
  "tasks": {
    "loop_dock": {
      "type": "navigate_pose",
      "pose": "dock_front",
      "repeat": 10,
      "pause_between": 3.0,
      "max_speed": 0.2
    }
  }
}
```

```bash
ros2 run autonomy_ros autonomy_cmd.py run-task loop_dock --feedback
```

---

## 二、直接子命令模式（单次调试）

不依赖 `tasks.json`，适合快速试验。

### 2.1 导航类 Action

**单点导航 `navigate-pose`**

```bash
ros2 run autonomy_ros autonomy_cmd.py navigate-pose \
  --x 1.0 --y 0.5 --yaw 0.0 --frame odom --feedback

# 限速
ros2 run autonomy_ros autonomy_cmd.py navigate-pose --x 2.0 --y 0.0 --max-speed 0.22
```

**多点导航 `navigate-through`**（需完整 goal JSON）

```bash
ros2 run autonomy_ros autonomy_cmd.py navigate-through \
  --goal-json $PKG/share/autonomy_ros/scripts/examples/navigate_through.json \
  --feedback
```

**展厅导览 `guided-tour`**

```bash
ros2 run autonomy_ros autonomy_cmd.py guided-tour \
  --goal-json $PKG/share/autonomy_ros/scripts/examples/guided_tour.json \
  --feedback
```

到展点且 `wait_for_continue: true` 时，另开终端：

```bash
ros2 run autonomy_ros autonomy_cmd.py continue-tour --tour-id hall_a --task-id <当前task_id>
```

**跟随 `follow`**

```bash
# 跟随指定位姿
ros2 run autonomy_ros autonomy_cmd.py follow --mode pose --x 1.5 --y 0.5 --duration 120

# 跟随检测 ID（需 Detection3DArray 话题，见 architecture.md）
ros2 run autonomy_ros autonomy_cmd.py follow --mode id --target-id person_01 --feedback
```

**回充 `dock`**

```bash
ros2 run autonomy_ros autonomy_cmd.py dock --dock-id dock_main --feedback
```

**遥操 `teleop`**（期间请自行发布 `/cmd_vel`）

```bash
ros2 run autonomy_ros autonomy_cmd.py teleop --duration 60 --preempt
```

### 2.2 管理类 Service

```bash
# 任务状态
ros2 run autonomy_ros autonomy_cmd.py status

# 取消当前任务
ros2 run autonomy_ros autonomy_cmd.py cancel --all

# 暂停 / 继续
ros2 run autonomy_ros autonomy_cmd.py pause --reason demo
ros2 run autonomy_ros autonomy_cmd.py resume

# 急停 / 解除
ros2 run autonomy_ros autonomy_cmd.py estop
ros2 run autonomy_ros autonomy_cmd.py estop --release

# 重定位（发布 /initialpose）
ros2 run autonomy_ros autonomy_cmd.py set-initial-pose --x 0 --y 0 --yaw 0

# 充电桩列表
ros2 run autonomy_ros autonomy_cmd.py list-docks

# 快速开启遥操模式（不启 Teleop Action）
ros2 run autonomy_ros autonomy_cmd.py set-teleop-mode --enable
ros2 run autonomy_ros autonomy_cmd.py set-teleop-mode --disable
```

---

## 三、子命令速查表

| 子命令 | 类型 | ROS 接口 |
|--------|------|----------|
| `list-config` | 配置 | 打印 `tasks.json` |
| `run-pose` | 配置 | 命名 pose + `repeat` |
| `run-task` | 配置 | `tasks` 表中的任务 |
| `run-mission` | 配置 | `missions` 序列 |
| `navigate-pose` | Action | `/autonomy/navigate_pose` |
| `navigate-through` | Action | `/autonomy/navigate_through` |
| `guided-tour` | Action | `/autonomy/guided_tour` |
| `follow` | Action | `/autonomy/follow` |
| `dock` | Action | `/autonomy/dock` |
| `teleop` | Action | `/autonomy/teleop` |
| `cancel` | Service | `/autonomy/cancel_task` |
| `status` | Service | `/autonomy/get_task_status` |
| `pause` / `resume` | Service | 暂停 / 继续 |
| `continue-tour` | Service | 导览「下一步」 |
| `skip-exhibit` | Service | 跳转展点 |
| `estop` | Service | `/autonomy/trigger_estop` |
| `set-initial-pose` | Service | 重定位 |
| `set-teleop-mode` | Service | 遥操开关 |
| `list-docks` | Service | 充电桩列表 |

---

## 四、典型工作流

### 4.1 Gazebo + TB3 快速导航

```bash
# 终端 1
ros2 launch autonomy_ros autonomy_stack.launch.py

# 终端 2：单次目标
ros2 run autonomy_ros autonomy_cmd.py navigate-pose --x 1.0 --y 0.0 --feedback

# 或使用配置：重复到 point_a 三次
export AUTONOMY_TASKS_CONFIG=$PKG/share/autonomy_ros/scripts/examples/tasks.json
ros2 run autonomy_ros autonomy_cmd.py run-task goto_a_x3 --feedback
```

### 4.2 监控任务与事件

```bash
ros2 topic echo /autonomy/status
ros2 topic echo /autonomy/events
ros2 run autonomy_ros autonomy_cmd.py status
```

### 4.3 紧急停止

```bash
ros2 run autonomy_ros autonomy_cmd.py estop
# 恢复
ros2 run autonomy_ros autonomy_cmd.py estop --release
ros2 run autonomy_ros autonomy_cmd.py resume
```

---

## 五、与 RViz / 原生 `ros2` 命令的关系

| 方式 | 说明 |
|------|------|
| `autonomy_cmd.py` | 封装好的子命令 + `tasks.json`，适合脚本与运维 |
| RViz `2D Goal Pose` | 发布 `goal_pose`，由节点内部发起导航（非本脚本） |
| `ros2 action send_goal ...` | 等价底层调用，见 [external_commands.md](external_commands.md) 文末 |

三者最终都调用同一套 `/autonomy/*` 服务端。

---

## 六、常见问题

| 现象 | 处理 |
|------|------|
| `Action server not available` | 确认 `autonomy_node` 已启动且 `enable_command:=true` |
| Goal rejected / NOT_AVAILABLE | 可能急停、或有任务占用；`cancel --all` 或 `estop --release` |
| 导航超时 | 增大 `--timeout` 或检查规划/地图/odom |
| `Unknown pose 'xxx'` | `list-config` 检查 `poses` 是否定义 |
| `guided_tour` 找不到文件 | `goal_file` 路径相对 **tasks.json 所在目录** |
| 帧不一致 | `planner.global_frame` 与 pose 的 `frame` 一致（仿真常用 `odom`） |

---

## 七、相关文件

| 路径 | 说明 |
|------|------|
| `scripts/autonomy_cmd.py` | CLI、配置、Goal 构建、ROS 客户端（单文件） |
| `scripts/examples/tasks.json` | 任务配置示例 |
| `scripts/examples/navigate_through.json` | 多点导航 goal 示例 |
| `scripts/examples/guided_tour.json` | 导览 goal 示例 |

---

## 八、延伸阅读

- [external_commands.md](external_commands.md) — Action/Service 字段与 `ros2 action send_goal` 示例  
- [architecture.md](architecture.md) — 数据流与 Bridge  
- [localization.md](localization.md) — `map` / `odom` 选型  
