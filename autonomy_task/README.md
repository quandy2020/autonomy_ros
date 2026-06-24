# autonomy_task

多机器人导航与数据采集编排包。管理 N 台机器人按导航点采集数据，支持距离阈值过滤、已采集点追踪、不可采集点剔除与状态持久化。

## 架构

```mermaid
flowchart TB
  CFG[collection_task.yaml] --> COORD[collection_coordinator_node]
  GRAPH[/robot1/habitat/graph] --> COORD
  STATE[collection_state.json] --> COORD

  COORD --> FILTER[WaypointFilter]
  COORD --> RC1[RobotClient robot1]
  COORD --> RC2[RobotClient robot2]
  COORD --> RCN[RobotClient robotN]

  RC1 --> NAV1[/robot1/navigate_to_pose]
  RC2 --> NAV2[/robot2/navigate_to_pose]
  RC1 --> REC1[/robot1/lerobot_bridge_node]
  RC2 --> REC2[/robot2/lerobot_bridge_node]

  COORD --> STATUS[~/collection_status JSON]
  COORD --> MARKERS[~/waypoint_markers]
```

## 导航点状态

| 状态 | 含义 | RViz 颜色 |
|------|------|-----------|
| `pending` | 待采集 | 黄色 |
| `in_progress` | 机器人正在前往 | 蓝色 |
| `collected` | 已采集 | 绿色 |
| `failed` | 超过最大重试次数 | 红色 |
| `skipped` | 被滤除（重复/越界/障碍/导航不可达） | 灰色 |
| `blocked` | 预留：被其他任务占用 | 紫色 |

## 滤除规则（WaypointFilter）

1. **dedupe_radius_m**：距离过近的重复导航点 → `skipped: duplicate_of:*`
2. **min_distance_m / max_distance_m**：相对机器人当前位姿过近/过远 → 不分配
3. **min_spacing_m**：距已采集点过近 → 不分配（`spacing_vs_collected: false` 时关闭，用于重复采集）
4. **max_spacing_m**（可选）：与最近已采集点距离过大 → 不分配（均匀覆盖）
5. **target_episodes / allow_revisit**：单场景重复采集；到达目标条数后停止（默认 2000）
6. **distance_buckets + assignment_strategy: bucket**：按直线距离分桶（1–2m、2–3m…）均衡采集
7. **require_navigable + map**：占据栅格上不可通行 → `skipped: occupied:*`
8. **持久化状态**：重启后已采集轨迹保留；`skipped` 节点不再分发

## 采集流程（每台机器人）

```
IDLE → RECORDING_BEFORE → NAVIGATING → RECORDING_AFTER → SAVING → IDLE
         (可选预热录制)      Nav2 导航      (到达后录制)    save_episode
```

- 导航成功：标记 `collected`，写入 `state_file`
- 导航失败（Nav2 abort / 超时 / 卡住）：取消当前 `navigate_to_pose`，**立即**分配下一个航点（`allow_revisit` 下同点可重试，超限才 `skipped`）

## 配置

主配置：`config/collection_task.yaml`

| 参数 | 说明 |
|------|------|
| `collection.min_distance_m` | 机器人到目标的最小分配距离 |
| `collection.max_distance_m` | 机器人到目标的最大分配距离 |
| `collection.min_spacing_m` | 已采集点之间的最小间隔（`spacing_vs_collected: false` 时不生效） |
| `collection.target_episodes` | 目标采集条数（0 = 采完所有可分配点即停） |
| `collection.allow_revisit` | 允许重复访问同一图节点（单场景大规模采集） |
| `collection.spacing_vs_collected` | 是否用历史采集位置做 min_spacing 过滤 |
| `collection.distance_buckets` | 距离分桶 `[[1,2],[2,3],...]`，配合 `assignment_strategy: bucket` |
| `collection.dedupe_radius_m` | 加载图节点时合并过近路点的半径（0 = 不去重） |
| `collection.assignment_strategy` | `nearest` / `farthest` / `dispersed` / `bucket` |
| `collection.parallel` | `true` 多机并行采集 |
| `state_file` | 采集进度 JSON 持久化路径 |
| `recording.enabled` | 是否调用 LeRobot `set_recording` / `save_episode` |
| `filter.require_navigable` | 是否根据占据地图滤除不可通行点 |

数据集路径（`dataset_root`、`dataset_repo_id`）由 `autonomy_lerobot/config/lerobot_collection.yaml` 与 launch 参数 `dataset_root` / `dataset_repo_id` 统一管理，不在本文件重复配置。

导航点：默认从 `/robot1/habitat/graph`（多机场景下 robot1 的 navmesh 图）加载；也可设置 `waypoint_source: grid` 用 `grid_sampling` 生成网格点。

## 启动

```bash
colcon build --packages-select autonomy_task autonomy_lerobot
source install/setup.bash

# 一键：多机 Habitat + Nav2 + LeRobot 录制 + 采集编排
ros2 launch autonomy_task multi_robot_collection.launch.py num_robots:=2 use_rviz:=true

# 可选参数
#   graph_topic:=/robot1/habitat/graph
#   dataset_root:=/workspace/autonomy/data/lerobot/collection
#   dataset_repo_id:=local/habitat_collection
#   recording_enabled:=true
#   clean_datasets_on_start:=false   # 默认 false：保留已有 LeRobot 数据并追加 episode
#   lerobot_delay:=12.0
#   coordinator_delay:=28.0
```

单独运行编排节点（数据集路径与 launch 保持一致）：

```bash
ros2 run autonomy_task collection_coordinator_node --ros-args \
  -p config_file:=$(ros2 pkg prefix autonomy_task)/share/autonomy_task/config/collection_task.yaml \
  -p num_robots:=2 \
  -p dataset_root:=/workspace/autonomy/data/lerobot/collection \
  -p dataset_repo_id:=local/habitat_collection
```

## 一键录制

`collection_coordinator` 提供全局录制控制，一次调用同时操作所有机器人。

**前置条件：**

- launch 已启动且 `collection_coordinator` 节点在运行
- 当前终端已 `source install/setup.bash`
- `recording.enabled: true`，各 `lerobot_bridge_node` 已启动（launch 默认 `recording_enabled:=true`）

**一键开始录制（所有机器人）：**

```bash
ros2 service call /collection_coordinator/start_recording std_srvs/srv/Trigger {}
```

**一键停止录制（所有机器人，自动保存 episode）：**

```bash
ros2 service call /collection_coordinator/stop_recording std_srvs/srv/Trigger {}
```

停止时会调用各机器人 `lerobot_bridge_node` 的 `set_recording(false)`，桥接节点自动执行 `save_episode()` 写入数据集。

确认服务已注册：

```bash
ros2 service list | grep collection_coordinator
# 应看到：
#   /collection_coordinator/start_recording
#   /collection_coordinator/stop_recording
```

单机器人手动控制（可选）：

```bash
ros2 service call /robot1/lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: true}"
ros2 service call /robot1/lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: false}"
ros2 service call /robot1/lerobot_bridge_node/save_episode std_srvs/srv/Trigger {}
```

## 数据落盘

LeRobot 数据集由 `autonomy_lerobot` 写入，多机时按机器人分子目录：

```
data/lerobot/collection/
├── robot1/
│   ├── meta/info.json       # total_episodes > 0 表示有数据
│   ├── data/chunk-*/        # parquet
│   └── videos/              # mp4
└── robot2/
```

采集进度（已采集航点）写入 `state_file`（默认 `data/collection/state.json`）。**重启 launch 会自动加载该文件**，已采集航点不会重复分配（累加采集）。

LeRobot 数据集默认**累加**：`clean_datasets_on_start` 默认为 `false`，重启后在已有 `robotN/` 目录上继续写新 episode。需要从零开始时：

```bash
ros2 launch autonomy_task multi_robot_collection.launch.py clean_datasets_on_start:=true
```

或手动删除：

```bash
rm -rf data/lerobot/collection/robot*
# 可选：同时重置采集进度
rm -f data/collection/state.json
```

验证落盘：

```bash
cat data/lerobot/collection/robot1/meta/info.json | grep total_episodes
find data/lerobot/collection/robot1 -name "*.parquet" -o -name "*.mp4"
```

若目录仅有 `meta/info.json` 而无 `tasks.parquet` / `data/` / `videos/`，说明上次录制中断，需清理后重试：

```bash
rm -rf data/lerobot/collection/robot*
```

## 话题与服务

节点名 `collection_coordinator`。下表 `~/...` 为节点私有名，完整路径为 `/collection_coordinator/...`。

| 完整路径 | 类型 | 说明 |
|----------|------|------|
| `/collection_coordinator/collection_status` | `std_msgs/String` | JSON 状态（路点、机器人、统计） |
| `/collection_coordinator/waypoint_markers` | `visualization_msgs/MarkerArray` | RViz 路点可视化 |
| `/collection_coordinator/start_recording` | `std_srvs/Trigger` | 一键开始录制（所有机器人） |
| `/collection_coordinator/stop_recording` | `std_srvs/Trigger` | 一键停止录制并保存（所有机器人） |
| `/{robot}/navigate_to_pose` | Nav2 Action | 导航 |
| `/{robot}/lerobot_bridge_node/set_recording` | `std_srvs/SetBool` | 单机开始/停止录制 |
| `/{robot}/lerobot_bridge_node/save_episode` | `std_srvs/Trigger` | 单机保存 episode |

## 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| `waiting for service to become available...` | launch 未运行、未 `source`、或代码未重新编译 | 重启 launch；`colcon build` 后 `source install/setup.bash` |
| 调 `/collection_coordinator/start_recording` 一直卡住无 response | 服务回调内同步 `call()` 子服务导致 executor 死锁 | 更新 `autonomy_task` 后 `colcon build` 并重启 launch；临时可分别调 `/robotN/lerobot_bridge_node/set_recording` |
| 调 `/collection_coordinator/start_recording` 无响应，但存在 `/start_recording` | 旧版服务名注册在根命名空间 | 重新编译 `autonomy_task` 并重启；或临时调用 `/start_recording` |
| `total_episodes: 0`，无 parquet/mp4 | 只调了 start 未调 stop，或 bridge 崩溃 | 成对调用 start/stop；检查 `lerobot_bridge_node` 日志 |
| 重启后 LeRobot 数据被清空 | 曾用 `clean_datasets_on_start:=true` 或打开失败时旧版会删库重建 | 使用默认 `clean_datasets_on_start:=false`；正常停止 launch 以便 `finalize()` 落盘 |
| `lerobot_bridge_node` 崩溃，`tasks.parquet` 缺失 | 录制中断，目录仅有 `meta/info.json` | 无 `data/` 时可 `rm -rf robot*`；有 `data/` 时勿删，重启 bridge 会尝试追加 |
| 采集立即结束，`near_done:g_XX` | `min_spacing_m` 过大，已采点 4m 排斥区盖住剩余 graph 节点 | 降低 `min_spacing_m`（默认已改为 1.5）；日志会显示 `collection exhausted` |
| 采集立即 `collection complete` | 机器人 odom 未就绪或航点被滤除 | 等待 habitat 启动完成；检查 `min_distance_m` / `max_distance_m` |

## 测试

```bash
cd src/autonomy_ros/autonomy_task
python3 -m pytest test/test_waypoint_filter.py -q
```

## 与现有包的关系

- **autonomy_ros**：提供多机 Nav2 launch 与 `navigate_to_pose`
- **autonomy_lerobot**：提供录制服务（可选）
- **autonomy_msgs**：任务消息（本包当前使用 Nav2 + 自定义 JSON 状态，后续可扩展为 action 客户端）
