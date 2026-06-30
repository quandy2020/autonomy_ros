# autonomy_lerobot

ROS2 Python 包：将 **Habitat 仿真** + **Nav2 导航** 数据对接 [Hugging Face LeRobot](https://github.com/huggingface/lerobot)。

## 环境依赖

**Docker 镜像默认未预装 LeRobot**，需在进入容器后执行一次安装脚本（约 10–15 分钟，含 PyTorch 等依赖）：

```bash
bash /workspace/autonomy/src/autonomy/docker/install/install_lerobot.sh
```

或手动安装：

```bash
pip3 install "lerobot==0.4.4"
pip3 install 'setuptools>=61,<80'   # 保持与 colcon 兼容
```

验证：

```bash
python3 -c "import lerobot; print(lerobot.__version__)"
lerobot-info
```

`autonomy_lerobot` 使用 `import lerobot` 写入 v3 本地数据集；未安装时会回退为 `.npz` 格式。回放与训练 CLI：

| 命令 | 用途 |
|------|------|
| `lerobot-dataset-viz` | 按 episode 可视化 RGB / 语义 / 向量字段（Rerun） |
| `lerobot-train` | 模仿学习训练 |
| `lerobot-replay` | 在真实机械臂上重放 `action`（非 Habitat） |
| `lerobot-edit-dataset` | 编辑 / 合并数据集 |
| `lerobot-info` | 查看环境与版本 |

本地数据集读写建议离线模式（避免误连 HuggingFace Hub）：

```bash
export HF_HUB_OFFLINE=1
```

> 已采集的 `.npz` 数据在安装 LeRobot 后不会自动转换；新录制的 episode 将写入标准 LeRobot 数据集格式。

## 包结构

```
autonomy_lerobot/
├── config.py           # ROS 参数
├── conversions.py      # ROS 消息 → numpy
├── observation.py      # 帧组装与字段常量
├── recorder.py         # LeRobot / npz 写入与 meta 修复
├── repo_id.py          # repo_id 规范化（namespace/repo_name）
├── dataset_cleanup.py  # 离线清理无效 episode / tmp 目录
└── node.py             # BridgeNode
```

## 构建与运行

colcon 构建 **ament_python** 包需要 `setuptools<80`（与 `--symlink-install` 兼容）。Docker/容器内若 `colcon build` 报 `--editable` / `--uninstall not recognized`：

```bash
pip3 install 'setuptools>=61,<80'
# 若曾 pip install 过本包，先卸载再 colcon 构建
pip3 uninstall -y autonomy-lerobot 2>/dev/null || true
rm -rf src/autonomy_ros/autonomy_tools/autonomy_lerobot/*.egg-info build/autonomy_lerobot
```

或使用工作区脚本：

```bash
bash src/autonomy_ros/scripts/ensure_colcon_python.sh
colcon build --packages-select autonomy_lerobot --symlink-install
source install/setup.bash
```

```bash
colcon build --packages-select autonomy_lerobot
source install/setup.bash

ros2 launch autonomy_lerobot lerobot_habitat_nav2.launch.py
```

## 录制控制

```bash
ros2 service call /lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: true}"
# 跑一段导航后保存（RGB/semantic 会写成 videos/.../*.mp4）
ros2 service call /lerobot_bridge_node/save_episode std_srvs/srv/Trigger {}
ros2 service call /lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: false}"
```

### 视频（MP4）说明

LeRobot 对 RGB、语义图等 `dtype: video` 字段默认写入 **MP4**（`videos/<key>/chunk-000/file-000.mp4`），不是长期保留 PNG。

| 阶段 | 磁盘内容 |
|------|----------|
| 录制中（`streaming_encoding: true`） | 直接写 MP4 |
| 录制中（`streaming_encoding: false`） | 临时 PNG 在 `images/`，`save_episode` 时转 MP4 并删除 PNG |
| `save_episode` 之后 | `videos/.../*.mp4` + parquet 元数据 |

相关参数（`lerobot_bridge.yaml`）：

- `video_vcodec: h264` — 编码器（推荐 `h264`，兼容性好）
- `streaming_encoding: true` — 录制时实时写 MP4，减少 `images/` 堆积
- `parallel_video_encoding: true` — 多路相机并行编码

若已有 `images/.../frame-*.png` 未转 MP4，可离线转换：

```bash
ros2 run autonomy_lerobot encode_dataset_videos.py \
  /workspace/autonomy/data/lerobot/habitat_nav2 --delete-png
```

---

## 录制数据来源

桥接节点按 `record_fps`（默认 10 Hz）采样，将各话题**最新一帧**对齐后写入数据集。

### Habitat 话题

| 参数 | 默认话题 | 消息类型 | 开关 |
|------|----------|----------|------|
| `rgb_topic` | `camera/rgb/image_raw` | `sensor_msgs/Image` | 始终 |
| `odom_topic` | `odom` | `nav_msgs/Odometry` | 始终 |
| `cmd_vel_topic` | `cmd_vel` | `geometry_msgs/Twist` | 始终 |
| `depth_topic` | `camera/depth/image_raw` | `sensor_msgs/Image` | `use_depth` 或 `record_depth` |
| `semantic_topic` | `camera/semantic/image_raw` | `sensor_msgs/Image` | `record_semantic` |
| `camera_info_topic` | `camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | `record_camera_info` |
| `map_topic` | `map` | `nav_msgs/OccupancyGrid` | `record_map` |
| `pointcloud_topic` | `semantic_pointcloud` | `sensor_msgs/PointCloud2` | `record_pointcloud` |

话题命名与 `autonomy_simulator/param/habitat.yaml` 一致。地图与点云在 Habitat 启动时以 `TRANSIENT_LOCAL` 发布一次（`occupancy_grid_rate_hz: 0.0`），桥接节点使用匹配 QoS 订阅。

### Nav2 规划话题

| 参数 | 默认话题 | 消息类型 | 含义 |
|------|----------|----------|------|
| `global_plan_topic` | `plan` | `nav_msgs/Path` | 全局规划路径 |
| `local_plan_topic` | `transformed_global_plan` | `nav_msgs/Path` | 局部跟踪路径（MPPI 默认；DWB/RPP 用 `local_plan`） |
| `global_costmap_topic` | `global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 全局代价地图 |
| `local_costmap_topic` | `local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 局部代价地图 |

由 `record_nav2` 统一开关（默认 `true`）。Nav2 须在桥接节点启动前完成初始化。

> **MPPI 说明**：`nav2_habitat_params.yaml` 使用 `nav2_mppi_controller::MPPIController`，该控制器**不发布** `local_plan`，而是发布 `transformed_global_plan`（当前正在跟踪的全局路径片段）。桥接节点会同时订阅 `local_plan` 与 `transformed_global_plan`，并只采用含路径点的消息。

---

## 帧数据结构（Frame）

每帧为字典，键名遵循 LeRobot `observation.*` 命名约定。

### 核心字段（始终录制）

#### `observation.images.rgb`

| 属性 | 值 |
|------|-----|
| dtype | `uint8` |
| shape | `(480, 640, 3)` |
| 来源 | Habitat RGB 相机，`rgb8` 编码 |
| 坐标系 | `camera_optical_frame` |

#### `observation.state`

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(5,)` |
| 坐标系 | `odom` |

| 索引 | 名称 | 单位 | 说明 |
|------|------|------|------|
| 0 | x | m | odom 系 X 位置 |
| 1 | y | m | odom 系 Y 位置 |
| 2 | yaw | rad | 航向角 |
| 3 | linear_x | m/s | 线速度 |
| 4 | angular_z | rad/s | 角速度 |

#### `action`

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(2,)` |

| 索引 | 名称 | 单位 | 说明 |
|------|------|------|------|
| 0 | linear_x | m/s | Nav2 → Habitat 线速度 |
| 1 | angular_z | rad/s | Nav2 → Habitat 角速度 |

#### `task`

| 属性 | 值 |
|------|-----|
| 类型 | `str` |
| 默认 | `"navigate to goal"` |

### Habitat 扩展字段

#### `observation.images.semantic`（`record_semantic:=true`）

| 属性 | 值 |
|------|-----|
| dtype | `uint8` |
| shape | `(480, 640, 3)` |
| 说明 | Habitat 语义 ID 着色后的 RGB 可视化图 |

#### `observation.images.depth`（`record_depth:=true` 或 `use_depth:=true`）

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(480, 640)` |
| 说明 | 深度图，单位米（Habitat `32FC1`） |

#### `observation.camera_info`（`record_camera_info:=true`）

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(11,)` |

| 索引 | 名称 | 说明 |
|------|------|------|
| 0 | fx | 焦距 fx（`CameraInfo.k[0]`） |
| 1 | fy | 焦距 fy（`CameraInfo.k[4]`） |
| 2 | cx | 主点 cx（`CameraInfo.k[2]`） |
| 3 | cy | 主点 cy（`CameraInfo.k[5]`） |
| 4 | width | 图像宽度 |
| 5 | height | 图像高度 |
| 6–10 | d0–d4 | 畸变系数（不足补零） |

#### `observation.map` + `observation.map_info`（`record_map:=true`）

**`observation.map`**

| 属性 | 值 |
|------|-----|
| dtype | `int8` |
| shape | `(H, W)`，随场景变化 |
| 值域 | `-1` 未知，`0` 空闲，`100` 占据 |

**`observation.map_info`**

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(6,)` |

| 索引 | 名称 | 说明 |
|------|------|------|
| 0 | resolution | 栅格分辨率 (m) |
| 1 | width | 栅格宽度 |
| 2 | height | 栅格高度 |
| 3 | origin_x | 地图原点 X |
| 4 | origin_y | 地图原点 Y |
| 5 | origin_yaw | 地图原点航向 |

#### `observation.pointcloud`（`record_pointcloud:=true`）

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(4096, 6)`，由 `max_pointcloud_points` 配置 |
| 列 | x, y, z, r, g, b |
| 说明 | 从 MP3D 语义 PLY 发布的场景点云，超出部分零填充 |

### Nav2 规划字段（`record_nav2:=true`）

#### `observation.global_plan` + `observation.global_plan_len`

**`observation.global_plan`**

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(512, 3)`，由 `max_path_waypoints` 配置 |
| 列 | x, y, yaw（`map` 系） |

**`observation.global_plan_len`**

| 属性 | 值 |
|------|-----|
| dtype | `float32` |
| shape | `(1,)` |
| 说明 | 有效路径点数量，其余为零填充 |

#### `observation.local_plan` + `observation.local_plan_len`

与全局路径格式相同。来源为 Nav2 `controller_server` 的局部跟踪路径：`transformed_global_plan`（MPPI，本项目默认）或 `local_plan`（DWB / RegulatedPurePursuit），坐标系为 `odom`。

#### `observation.global_costmap` + `observation.global_costmap_info`

与 `observation.map` 相同格式，来源为 Nav2 全局代价地图（`map` 系，含膨胀层）。

#### `observation.local_costmap` + `observation.local_costmap_info`

与 `observation.map` 相同格式，来源为 Nav2 局部代价地图（`odom` 系滚动窗口，默认 4 m × 4 m）。

---

## 存储格式

### LeRobot 数据集（默认）

- 路径：`dataset_root`（单机默认 `~/.cache/lerobot/habitat_nav2`；多机采集见 [多机采集数据集](#多机采集数据集dataleobotcollection)）
- ID：`dataset_repo_id`（单机 `local/habitat_nav2`；多机 `local/habitat_collection_robotN`）
- 格式：LeRobot **v3.0**（`meta/` + `data/*.parquet` + `videos/*.mp4`）
- 首帧自动推断各字段的 feature schema（视频 / 图像 / 向量）
- 节点退出时调用 `finalize()` 关闭写入器
- 打开已有数据集时自动修复 `meta/info.json` 与 `meta/episodes` 不一致（见 `recorder.py`）

### npz 回退（仅 `import lerobot` 失败时）

若 Python 环境未装 LeRobot，`recorder.py` 会退化为 `dataset_root/episode_XXXX_*/episode.npz`。
**本仓库 Docker 环境不会走此路径。**

路径：`dataset_root/episode_XXXX_YYYYMMDD_HHMMSS/`

**`episode.npz`**：所有字段按时间维 `stack`，键名将 `.` 替换为 `_`：

```
observation_images_rgb          (T, 480, 640, 3)  uint8
observation_state               (T, 5)             float32
observation_images_semantic     (T, 480, 640, 3)  uint8
observation_camera_info         (T, 11)            float32
observation_map                 (T, H, W)          int8
observation_map_info            (T, 6)             float32
observation_pointcloud          (T, 4096, 6)       float32
observation_global_plan         (T, 512, 3)        float32
observation_global_plan_len     (T, 1)             float32
observation_local_plan          (T, 512, 3)        float32
observation_local_plan_len      (T, 1)             float32
observation_global_costmap      (T, H, W)          int8
observation_global_costmap_info (T, 6)             float32
observation_local_costmap       (T, H, W)          int8
observation_local_costmap_info  (T, 6)             float32
action                          (T, 2)             float32
task                            标量 str
fps                             标量 int
```

**`meta.json`**：`repo_id`、`fps`、`num_frames`、`keys`（实际写入的字段列表）。

**加载示例**：

```python
import numpy as np

data = np.load("episode_0000_.../episode.npz")
rgb = data["observation_images_rgb"]       # (T, H, W, 3)
plan = data["observation_global_plan"]     # (T, 512, 3)
plan_len = data["observation_global_plan_len"]  # (T, 1)
```

---

## 多机采集数据集（`data/lerobot/collection`）

`multi_robot_collection.launch.py` 将每台机器人数据写入独立子目录；`repo_id` 为
`local/habitat_collection_robot{N}`（LeRobot 要求恰好一个 `/`）。

### 目录结构

以 `robot1` 为例（LeRobot v3.0 本地格式）：

```
data/lerobot/collection/
├── robot1/
│   ├── meta/
│   │   ├── info.json          # 特征 schema、fps、total_episodes
│   │   ├── episodes/          # 每条 episode 的帧数、视频时间戳
│   │   └── tasks.parquet
│   ├── data/chunk-000/        # 向量字段 parquet（state、action、plan…）
│   │   └── file-000.parquet
│   └── videos/
│       ├── observation.images.rgb/chunk-000/file-*.mp4
│       └── observation.images.semantic/chunk-000/file-*.mp4
├── robot2/
...
```

| 机器人 | 本地路径 | `repo_id` |
|--------|----------|-----------|
| robot1 | `data/lerobot/collection/robot1` | `local/habitat_collection_robot1` |
| robot2 | `data/lerobot/collection/robot2` | `local/habitat_collection_robot2` |
| … | `.../robotN` | `local/habitat_collection_robotN` |

当前采集配置（`lerobot_collection.yaml`）主要录制 **RGB + depth + semantic + camera_info + state/action**；
较早 episode 可能仍含 map / plan 等字段（取决于录制时的 `record_*` 开关）。

### 查看统计

```bash
source install/setup.bash
python3 -m autonomy_task.collection_stats_cli \
  --state /workspace/autonomy/data/collection/state.json \
  --dataset-root /workspace/autonomy/data/lerobot/collection
```

输出协调器 `state.json` 轨迹条数与各机器人 LeRobot `meta/info.json` 帧数。

### 清理无效数据

删除 `tmp*` 临时目录、帧数异常 episode（默认 <5 或 >600 帧），并重编号为连续 `0..N-1`：

```bash
# 预览
ros2 run autonomy_lerobot cleanup_lerobot_dataset.py --dry-run

# 执行（默认根目录 data/lerobot/collection）
ros2 run autonomy_lerobot cleanup_lerobot_dataset.py
```

或直接：

```bash
python3 -m autonomy_lerobot.dataset_cleanup --dry-run
python3 -m autonomy_lerobot.dataset_cleanup
```

---

## 数据回放与使用

以下命令在 Docker / 工作区内可直接运行（LeRobot 已预装）。
本地数据集**不会**从 HuggingFace 拉取，请先：

```bash
export HF_HUB_OFFLINE=1
```

### 1. Rerun 可视化（推荐：RGB + 语义）

按 episode 回放相机视频与向量字段，支持本地窗口或导出 `.rrd`：

```bash
# 交互查看 robot1 的第 0 条 episode
lerobot-dataset-viz \
  --repo-id local/habitat_collection_robot1 \
  --root /workspace/autonomy/data/lerobot/collection/robot1 \
  --episode-index 0

# 无显示器时导出 Rerun 文件，拷贝到本机用 rerun 打开
lerobot-dataset-viz \
  --repo-id local/habitat_collection_robot1 \
  --root /workspace/autonomy/data/lerobot/collection/robot1 \
  --episode-index 0 \
  --save 1 \
  --output-dir /tmp/lerobot_viz
# 本机: rerun /tmp/lerobot_viz/local_habitat_collection_robot1_episode_0.rrd
```

远程机器有数据、本机只有显示器时，可用 `--mode distant`（见 `lerobot-dataset-viz --help`）。

### 2. Python API 按帧读取

适合自定义分析、画图或导出：

```python
import os
os.environ["HF_HUB_OFFLINE"] = "1"

from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = "/workspace/autonomy/data/lerobot/collection/robot1"
repo_id = "local/habitat_collection_robot1"

ds = LeRobotDataset(repo_id=repo_id, root=root)
print(f"episodes={ds.num_episodes} fps={ds.fps}")

# 单条 episode
ds_ep0 = LeRobotDataset(repo_id=repo_id, root=root, episodes=[0])
for i in range(len(ds_ep0)):
    frame = ds_ep0[i]
    rgb = frame["observation.images.rgb"]       # (C, H, W) float tensor
    action = frame["action"]                    # (2,) linear_x, angular_z
    state = frame["observation.state"]          # (5,) x, y, yaw, vx, wz
    ep = int(frame["episode_index"])
    t = float(frame["timestamp"])
```

合并多机数据训练时，分别构造多个 `LeRobotDataset`，或使用 LeRobot 的多数据集加载接口（见官方文档）。

### 3. 直接播放 MP4

视频路径见 `meta/episodes` 中 `videos/<key>/chunk_index` 与 `file_index`，或直接查看：

```bash
ffplay -autoexit /workspace/autonomy/data/lerobot/collection/robot1/videos/observation.images.rgb/chunk-000/file-000.mp4
ffplay -autoexit /workspace/autonomy/data/lerobot/collection/robot1/videos/observation.images.semantic/chunk-000/file-000.mp4
```

部分机器人将多条 episode 合并在同一 MP4 中，时间范围由 `meta/episodes` 的
`from_timestamp` / `to_timestamp` 指定；精确对齐请用 **§1 Rerun** 或 **§2 API**。

### 4. 用于模仿学习训练

LeRobot 训练入口（示例，按官方 CLI 调整 policy / batch）：

```bash
export HF_HUB_OFFLINE=1

lerobot-train \
  --dataset.repo_id=local/habitat_collection_robot1 \
  --dataset.root=/workspace/autonomy/data/lerobot/collection/robot1 \
  --policy.type=act \
  --output_dir=/tmp/lerobot_train_robot1
```

多机合并训练可为每台机器人指定多个 `--dataset.repo_id` / `--dataset.root`（以当前 `lerobot` 版本 CLI 为准，运行 `lerobot-train --help` 查看）。

### 5. 与仿真的关系

| 工具 | 用途 |
|------|------|
| `lerobot-dataset-viz` | **离线回放**已录制的图像与状态 |
| `lerobot-replay` | 在**真实机械臂**上重放数据集中的 `action`（非 Habitat） |
| `lerobot_bridge_node` | 仿真运行时**录制**新数据 |

本仓库采集的是 Nav2 导航轨迹（`action` = `cmd_vel`），**不能**直接用 `lerobot-replay` 在 Habitat 里“一键复现”整条 episode；若要在仿真中复现，需自行读取 `action` 序列并发布到 `/robotN/cmd_vel`，或训练策略后在线推理。

### 6. 离线补编码

若磁盘上仍有 `images/.../frame-*.png` 未转 MP4：

```bash
ros2 run autonomy_lerobot encode_dataset_videos.py \
  /workspace/autonomy/data/lerobot/collection/robot1 --delete-png
```

---

## 数据流

```
Habitat                          Nav2
  ├─ camera/rgb/image_raw          ├─ plan (全局路径)
  ├─ camera/semantic/image_raw     ├─ local_plan
  ├─ camera/rgb/camera_info        ├─ global_costmap/costmap
  ├─ camera/depth/image_raw        └─ local_costmap/costmap
  ├─ odom
  ├─ map
  ├─ semantic_pointcloud
  └─ cmd_vel ◄─────────────────────── (Nav2 输出)
                │
         lerobot_bridge_node
         (record_fps 采样对齐)
                │
           LeRobot Dataset v3
```

---

## 配置参数

见 `config/lerobot_bridge.yaml`。可用 `record_*` 开关按需关闭不需要的字段以减小数据量：

```yaml
use_depth: true          # 订阅 depth 话题（推理/观测）
record_depth: true       # 写入 observation.images.depth
record_semantic: true
record_map: true
record_pointcloud: true
record_camera_info: true
record_nav2: true
max_path_waypoints: 512
max_pointcloud_points: 4096
```
