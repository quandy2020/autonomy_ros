# autonomy_lerobot

将 Habitat 仿真数据写入 [LeRobot](https://github.com/huggingface/lerobot) v3 数据集。桥接节点 `lerobot_bridge_node` 按 `record_fps` 采样 ROS 话题最新一帧，对齐后落盘。

## 快速开始

```bash
# 1. 安装 LeRobot（Docker 内需执行一次）
bash /workspace/autonomy/src/autonomy/docker/install/install_lerobot.sh

# 2. 构建（修改本包后需重新 source）
colcon build --packages-select autonomy_lerobot && source install/setup.bash

# 3. 启动（二选一）
ros2 launch autonomy_lerobot lerobot_habitat_nav2.launch.py   # 单机 Nav2
# 或 multi_robot_collection.launch.py（见 autonomy_task）

# 4. 录制
ros2 service call /lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: true}"
ros2 service call /lerobot_bridge_node/save_episode std_srvs/srv/Trigger {}
ros2 service call /lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: false}"
```

本地数据集建议 `export HF_HUB_OFFLINE=1`。未安装 LeRobot 时回退为 `.npz`（Docker 环境通常不走此路径）。

构建报错 `--editable` 时：`pip3 install 'setuptools>=61,<80'` 后重试 colcon。

---

## 录什么？（两种格式）

由 `dataset_format` 与 `record_*` 开关决定。另自动写入 LeRobot 元数据：`timestamp`、`frame_index`、`episode_index`、`index`、`task_index`。

| | `habitat_nav2` | `jdrobot` |
|---|----------------|-----------|
| 用途 | 单机 Habitat + Nav2 | 多机采集 / kujiale 兼容 |
| 配置 | `config/lerobot_bridge.yaml` | `config/lerobot_collection.yaml` |
| 帧率 | 10 Hz | 20 Hz |
| `repo_id` | `local/habitat_nav2` | `local/habitat_collection_robotN` |
| `action` | `cmd_vel` (2,) | odom 位姿 4×4 (16,) |
| 视频编码 | h264 | av1 |
| **默认数据键** | **18** | **7** |

### `habitat_nav2` — 18 键（默认全开）

| 类别 | 字段 | shape / 说明 |
|------|------|--------------|
| 视频 | `observation.images.rgb`、`observation.images.semantic` | (480, 640, 3) |
| 图像 | `observation.images.depth` | (480, 640) float，米 |
| 状态 | `observation.state` | (5,) x, y, yaw, vx, wz |
| 动作 | `action` | (2,) linear_x, angular_z |
| 相机 | `observation.camera_info` | (11,) fx,fy,cx,cy,w,h,d0–d4 |
| 地图 | `observation.map` + `map_info` | 占用栅格 + 6 维元信息 |
| 点云 | `observation.pointcloud` | (4096, 6) x,y,z,r,g,b，**相机系** |
| Nav2 | `global/local_plan` + `_len` | (512, 3) + 有效点数 |
| Nav2 | `global/local_costmap` + `_info` | 代价地图 + 元信息 |
| 文本 | `task` | 默认 `"navigate to goal"` |

订阅 12 路话题；写帧前需 **RGB + odom + depth + camera_info**（点云由 depth 反投影）。

### `jdrobot` — 7 键

| 字段 | 说明 |
|------|------|
| `observation.images.rgb` | RGB 视频 |
| `observation.images.depth` | 深度灰度视频（非原始 float） |
| `observation.camera_intrinsic` | K 矩阵 (9,) |
| `observation.camera_extrinsic` | 单位阵 (16,) |
| `action` | odom 位姿矩阵 (16,) |
| `observation.pointcloud` | (4096, 6)，相机系 |
| `task` | 任务描述 |

订阅 RGB、depth、camera_info、odom。**无** state、语义、地图、Nav2。

### 点云

| `pointcloud_from_depth` | 行为 |
|-------------------------|------|
| `true`（默认） | depth + camera_info 反投影，相机光学系 |
| `false` | 订阅 `semantic_pointcloud`（地图系静态点云） |

参数：`max_pointcloud_points`（4096）、`pointcloud_stride`（4）、`depth_min_m` / `depth_max_m`。

### 数据量（60 s episode）

| 格式 | 帧数 | 体积主要来自 |
|------|------|--------------|
| habitat_nav2 @ 10 Hz | 600 | 双路视频 + depth + 栅格/路径 |
| jdrobot @ 20 Hz | 1200 | 双路视频 + 点云 parquet |

用 `record_*: false` 可关掉不需要的字段（如 `record_nav2` 少 8 键）。

---

## 存储布局

LeRobot v3：`meta/` + `data/*.parquet` + `videos/*.mp4`。

数据路径由 `autonomy_lerobot/config/data_paths.yaml` 解析（`volume_root` → `{volume_root}/data/lerobot/collection`）。查看本机路径：

```bash
python3 -c "from autonomy_lerobot.data_paths import lerobot_collection_root; print(lerobot_collection_root())"
```

多机目录示例（`robot1` 在数据盘上）：

```
data/lerobot/collection/robot1/
├── meta/info.json
├── data/chunk-000/file-000.parquet
└── videos/observation.images.{rgb,depth}/...
```

统计与清理：

```bash
python3 -m autonomy_task.collection_stats_cli \
  --state data/collection/state.json --dataset-root data/lerobot/collection
ros2 run autonomy_lerobot cleanup_lerobot_dataset --dry-run
```

---

## 回放与训练

数据默认写在 **`/mnt/data4t/data/lerobot/collection/robotN/`**（由 `config/data_paths.yaml` 的 `volume_root` 决定），**不是** `workspace/autonomy/data/`（除非已做符号链接）。

```bash
export HF_HUB_OFFLINE=1

# 推荐：自动解析 collection 路径（需先 colcon build 并 source install/setup.bash）
ros2 run autonomy_lerobot lerobot_dataset_viz --robot robot1 --episode-index 0

# Docker 内 GUI「Save」不可用；播放结束后会自动导出 .rrd 到：
#   /mnt/data4t/data/lerobot/viz/<repo_id>_episode_<N>.rrd
# 也可显式导出（不弹 viewer）：
ros2 run autonomy_lerobot lerobot_dataset_viz --robot robot1 --episode-index 0 \
  --save-dir /mnt/data4t/data/lerobot/viz

# 或在宿主机浏览器查看（容器内 distant 模式）：
ros2 run autonomy_lerobot lerobot_dataset_viz --robot robot1 --episode-index 0 --mode distant

# 或直接调用模块（不经过 ros2 run）
python3 -m autonomy_lerobot.dataset_viz --robot robot1 --episode-index 0

# 或手动指定绝对路径
lerobot-dataset-viz --repo-id local/habitat_collection_robot1 \
  --root /mnt/data4t/data/lerobot/collection/robot1 --episode-index 0

# Python 读取
python3 -c "
import os; os.environ['HF_HUB_OFFLINE']='1'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('local/habitat_collection_robot1',
    root='/mnt/data4t/data/lerobot/collection/robot1')
print(ds.num_episodes, ds.fps)
"

# 训练（示例）
lerobot-train --dataset.repo_id=local/habitat_collection_robot1 \
  --dataset.root=/mnt/data4t/data/lerobot/collection/robot1 --policy.type=act
```

若 `ros2 run` 报 **No executable found**，说明未构建或未 `source install/setup.bash`；若报 **401 / Repository Not Found**，说明 `--root` 路径错误或未设 `HF_HUB_OFFLINE=1`。

---

## 常用参数

配置文件：`config/lerobot_bridge.yaml`、`config/lerobot_collection.yaml`。

```yaml
dataset_format: habitat_nav2   # 或 jdrobot
record_fps: 10.0
record_depth / record_semantic / record_map / record_nav2: true  # habitat 按需
record_pointcloud: true
pointcloud_from_depth: true
max_pointcloud_points: 4096
video_vcodec: h264             # jdrobot 用 av1
```

话题默认值与 `autonomy_simulator/param/habitat.yaml` 对齐（`camera/rgb/image_raw`、`odom`、`map` 等）。MPPI 控制器发布 `transformed_global_plan` 而非 `local_plan`，桥接节点两者均订阅。
