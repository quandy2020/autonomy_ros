# autonomy_slam

基于 `autonomy` atlas 模块的 ROS 2 视觉 SLAM 封装包。支持单目、双目、RGB-D，可视化使用 **rviz2**。

可与同工作空间中的 [`autonomy_internnav`](../autonomy_planning/autonomy_internnav/)（NavDP 视觉导航）联用：SLAM 提供定位与地图，InternNav 根据 RGB-D 与目标点输出 `cmd_vel`。

## 目录

- [环境要求](#环境要求)
- [编译安装](#编译安装)
- [快速上手](#快速上手)
- [与 autonomy_internnav 联用](#与-autonomy_internnav-联用)
- [离线回放（rosbag2）](#离线回放rosbag2)
- [Composable 节点](#composable-节点)
- [rviz2 可视化](#rviz2-可视化)
- [话题与参数](#话题与参数)
- [命令行参数](#命令行参数)
- [包结构](#包结构)

## 环境要求

- ROS 2（Humble 或更高版本）
- 已编译安装的 `autonomy` 包（提供 atlas SLAM 核心）
- OpenCV、Eigen3、yaml-cpp
- 词袋文件（ORB vocab）与相机/SLAM 配置文件（YAML）

## 编译安装

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select autonomy autonomy_slam
source install/setup.bash
```

若需与 NavDP 视觉导航联用，一并编译 `autonomy_internnav` 并安装 Python 依赖：

```bash
colcon build --packages-select autonomy autonomy_slam autonomy_internnav --symlink-install
source install/setup.bash
bash src/autonomy_ros/autonomy_planning/autonomy_internnav/scripts/install_deps.sh
```

仅修改本包时：

```bash
colcon build --packages-select autonomy_slam
source install/setup.bash
```

## 快速上手

### 1. 准备词袋文件

将 `orb_vocab.fbow` 放入 `vocab/` 目录（参见 `vocab/README.md`），或通过 launch 参数 `vocab_file` 指定路径。

### 2. 选择相机模式并启动

**单目（Mono）**

```bash
ros2 launch autonomy_slam slam_mono.launch.py
# 或
ros2 launch autonomy_slam slam.launch.py mode:=mono
```

**双目（Stereo）**

```bash
ros2 launch autonomy_slam slam_stereo.launch.py
# 原始图像需校正时
ros2 launch autonomy_slam slam_stereo.launch.py rectify:=true
```

**RGB-D**

```bash
ros2 launch autonomy_slam slam_rgbd.launch.py
```

默认同时启动 rviz2。仅启动 SLAM：

```bash
ros2 launch autonomy_slam slam.launch.py mode:=mono launch_rviz:=false
```

### 3. AIST Living Lab 示例（`data/aist_living_lab`）

先下载数据与词袋：

```bash
cd src/autonomy_ros/autonomy_slam/vocab
./download_vocab.sh
./download_aist_example.sh
colcon build --packages-select autonomy_slam
source install/setup.bash
```

对应 [stella_vslam 教程](https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html) 的三步流程（`run_video_slam`，与 `run_video_slam` CLI 一致）：

```bash
# 建图（aist_living_lab_1/video.mp4 → aist_living_lab_1_map.msg）
ros2 launch autonomy_slam aist_mapping.launch.py

# 定位（aist_living_lab_2，加载上一步地图）
ros2 launch autonomy_slam aist_localization.launch.py

# 时序定位
ros2 launch autonomy_slam aist_localization_temporal.launch.py
```

等价的原生 CLI（教程同款参数）：

```bash
ros2 run autonomy_slam run_video_slam \
  -v install/autonomy_slam/share/autonomy_slam/vocab/orb_vocab.fbow \
  -m data/aist_living_lab/aist_living_lab_1/video.mp4 \
  -c install/autonomy_slam/share/autonomy_slam/config/atlas/aist_equirectangular.yaml \
  --frame-skip 3 --no-sleep \
  --map-db-out data/aist_living_lab/aist_living_lab_1_map.msg

ros2 run autonomy_slam run_video_slam --disable-mapping \
  -v .../orb_vocab.fbow \
  -m data/aist_living_lab/aist_living_lab_2/video.mp4 \
  -c .../aist_equirectangular.yaml \
  --frame-skip 3 --no-sleep \
  --map-db-in data/aist_living_lab/aist_living_lab_1_map.msg
```

或使用脚本 `scripts/aist_tutorial.sh mapping|localization|temporal`。

| stella_vslam 教程 | autonomy_slam |
|-------------------|---------------|
| `./run_video_slam -v ... -m video.mp4 -c equirectangular.yaml --frame-skip 3 --no-sleep --map-db-out map.msg` | `ros2 run autonomy_slam run_video_slam`（同上参数） |
| `--disable-mapping --map-db-in map.msg` | `aist_localization.launch.py` 或 `--disable-mapping` |
| `--temporal-mapping --map-db-in map.msg` | `aist_localization_temporal.launch.py` |

通用入口（可改 `task`、`sequence`、`frame_skip` 等）：

```bash
ros2 launch autonomy_slam aist_slam.launch.py \
  task:=mapping sequence:=aist_living_lab_1 frame_skip:=3
```

| Launch 文件 | 说明 |
|-------------|------|
| `aist_mapping.launch.py` | 序列 1 建图，输出 `data/aist_living_lab/aist_living_lab_1_map.msg` |
| `aist_localization.launch.py` | 序列 2 纯定位（`--disable-mapping`） |
| `aist_localization_temporal.launch.py` | 序列 2 时序定位（`--temporal-mapping`） |
| `aist_slam.launch.py` | 可配置 `task` / `sequence` 的通用入口 |

使用全景相机配置 `config/atlas/aist_equirectangular.yaml`，由 `run_video_slam` 直接读取 MP4（不经过 ROS/cv_bridge）。

### 4. 常用 launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `mono` | `mono` / `stereo` / `rgbd` |
| `vocab_file` | `vocab/orb_vocab.fbow` | ORB 词袋路径 |
| `atlas_config` | `config/atlas/<mode>.yaml` | atlas SLAM 配置 |
| `image_topic` | `camera/image_raw` | 单目图像话题 |
| `left_topic` / `right_topic` | `camera/left/right/image_raw` | 双目话题 |
| `color_topic` / `depth_topic` | `camera/color/depth/image_raw` | RGB-D 话题 |
| `map_db_in` / `map_db_out` | 空 | 加载/保存地图 |
| `launch_rviz` | `true` | 是否启动 rviz2 |
| `use_sim_time` | `false` | 仿真时间 |

**离线 rosbag2 回放：**

```bash
ros2 launch autonomy_slam slam.launch.py \
  mode:=mono \
  use_offline:=true \
  bag_path:=/path/to/rosbag2_dir \
  launch_rviz:=true
```

### 5. 自定义配置

- ROS 节点参数：`config/mono.yaml`、`config/stereo.yaml`、`config/rgbd.yaml`
- Atlas 相机/算法参数：`config/atlas/mono.yaml`、`stereo.yaml`、`rgbd.yaml`（示例内参，请按实际相机修改）

示例：指定自定义 atlas 配置与话题

```bash
ros2 launch autonomy_slam slam.launch.py \
  mode:=stereo \
  atlas_config:=/path/to/my_stereo.yaml \
  left_topic:=/stereo/left/image \
  right_topic:=/stereo/right/image
```

## 与 autonomy_internnav 联用

[`autonomy_internnav`](../autonomy_planning/autonomy_internnav/) 提供 NavDP 点目标视觉导航：订阅 RGB-D、`odom` 与 `goal_pose`，发布 `cmd_vel`、规划路径与可视化 marker。训练代码位于 `autonomy_internnav/train/`（由 InternNav 整合而来）。

### 数据流

```text
相机 RGB-D ──┬──► autonomy_slam（可选）──► map / TF / camera_pose
             │
             └──► internnav_node（NavDP）──► cmd_vel / navdp/plan
odom ──────────────────────────────────────► internnav_node
goal_pose（RViz 2D Goal）──────────────────► internnav_node
```

- **SLAM**：建图、重定位、发布 `map` → `odom` TF 与 `~/camera_pose`
- **InternNav**：在机器人坐标系下根据视觉输入与目标点生成轨迹与速度指令

### 默认话题对照

`autonomy_internnav` 默认配置见 `autonomy_planning/autonomy_internnav/config/navdp.yaml`：

| InternNav 参数 | 默认值 | 说明 |
|----------------|--------|------|
| `rgb_topic` | `camera/rgb/image_raw` | RGB 图像 |
| `depth_topic` | `camera/depth/image_raw` | 深度图 |
| `camera_info_topic` | `camera/rgb/camera_info` | 相机内参 |
| `odom_topic` | `odom` | 里程计 |
| `goal_topic` | `goal_pose` | 目标位姿 |
| `cmd_vel_topic` | `cmd_vel` | 速度输出 |

本包 RGB-D 模式默认订阅 `camera/color/image_raw`、`camera/depth/image_raw`。与 InternNav 联用时，请统一话题名（launch 重映射或修改 `navdp.yaml`）。

| 场景 | RGB-D 来源 | `odom` 来源 |
|------|------------|-------------|
| Habitat 仿真 | `autonomy_simulator` 桥接 | 仿真器 `odom` |
| Gazebo TB3 | 相机插件 | `robot_state_publisher` / 仿真 odom |
| 实机 + SLAM | 相机驱动 | `autonomy_slam` 的 `~/camera_pose` 或轮式里程计 |

### Habitat + NavDP 示例（无需 SLAM）

Habitat 桥接话题与 `navdp.yaml` 默认一致，可直接启动 NavDP：

```bash
# 终端 1：Habitat 仿真
ros2 launch autonomy_simulator habitat.launch.py

# 终端 2：NavDP 导航（需 GPU 与 checkpoint，见 weights/README.md）
ros2 launch autonomy_internnav internnav.launch.py use_sim_time:=false
```

在 RViz 中使用 **2D Goal Pose** 设置 `goal_pose`。

### SLAM + NavDP 示例

实机或 rosbag 回放时，SLAM 负责定位，InternNav 负责视觉导航：

```bash
# 终端 1：RGB-D SLAM（按实际相机话题调整 launch 参数）
ros2 launch autonomy_slam slam_rgbd.launch.py \
  color_topic:=camera/rgb/image_raw \
  depth_topic:=camera/depth/image_raw \
  launch_rviz:=false

# 终端 2：NavDP（若 odom 来自 SLAM，将 camera_pose 转为 odom 或配置 odom 源）
ros2 launch autonomy_internnav internnav.launch.py
```

若 SLAM 仅发布 `~/camera_pose` 而无 `odom`，可增加 relay 节点将 `nav_msgs/Odometry` 转发到 `/odom`，或在 `navdp.yaml` 中将 `odom_topic` 改为 SLAM 输出话题。

### NavDP 权重与训练

| 步骤 | 说明 |
|------|------|
| 推理权重 | 放置于 `autonomy_internnav/weights/navdp-cross-modal.ckpt`，或设置 `NAVDP_CHECKPOINT` |
| 训练 | `bash autonomy_internnav/scripts/train/start_train.sh --name my_run --gpus 1` |
| 训练产出 | `checkpoints/<run_name>/ckpts/navdp.ckpt`，可复制到 `weights/` 用于推理 |

详见 [`autonomy_internnav/weights/README.md`](../autonomy_planning/autonomy_internnav/weights/README.md)。

### 6. 手动启动（不使用 launch）

```bash
ros2 run autonomy_slam run_slam \
  --vocab /path/to/orb_vocab.fbow \
  --config /path/to/config.yaml
```

## 视频文件 SLAM（run_video_slam）

与 [stella_vslam Simple Tutorial](https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html) 中 `run_video_slam` 参数一致，直接读取 MP4，不依赖 ROS 图像话题：

```bash
ros2 run autonomy_slam run_video_slam -h

ros2 run autonomy_slam run_video_slam \
  -v /path/to/orb_vocab.fbow \
  -m /path/to/video.mp4 \
  -c /path/to/aist_equirectangular.yaml \
  --frame-skip 3 --no-sleep \
  --map-db-out /path/to/map.msg
```

## 离线回放（rosbag2）

无需单独播放 bag，可直接从 rosbag2 目录读取并跑 SLAM：

```bash
ros2 run autonomy_slam run_slam_offline \
  -v /path/to/orb_vocab.fbow \
  -c /path/to/config.yaml \
  -b /path/to/rosbag2_dir
```

常用选项：

```bash
# 指定图像话题（单目默认 camera/image_raw）
ros2 run autonomy_slam run_slam_offline ... --camera camera/image_raw

# 加速回放（不等待真实时间间隔）
ros2 run autonomy_slam run_slam_offline ... --no-sleep

# 保存轨迹与地图
ros2 run autonomy_slam run_slam_offline ... \
  --eval-log-dir /tmp/slam_eval \
  -o /tmp/map.msg
```

离线回放时也可在另一终端启动 `ros2 launch autonomy_slam rviz.launch.py` 实时查看位姿与关键帧。

## Composable 节点

本包提供可组合节点 `autonomy_slam::System`，可嵌入 component container：

```bash
ros2 component standalone autonomy_slam autonomy_slam::System \
  --ros-args \
  -p vocab_file_path:=/path/to/orb_vocab.fbow \
  -p setting_file_path:=/path/to/config.yaml
```

或通过 `config/param.yaml` 加载参数。

## rviz2 可视化

```bash
ros2 launch autonomy_slam rviz.launch.py
```

默认显示（节点名 `run_slam`）：

| 显示项 | 话题 / 坐标系 |
|--------|----------------|
| 相机位姿 | `/run_slam/camera_pose` (`nav_msgs/Odometry`) |
| 关键帧 | `/run_slam/keyframes` (`geometry_msgs/PoseArray`) |
| 2D 关键帧 | `/run_slam/keyframes_2d` (`geometry_msgs/PoseArray`) |
| TF | `map` → `odom` → `camera_frame`（`publish_tf:=true` 时） |

**重定位**：在 rviz2 中使用 **2D Pose Estimate** 工具，向 `/initialpose` 发布初始位姿。

仿真时间：

```bash
ros2 launch autonomy_slam rviz.launch.py use_sim_time:=true
```

## 话题与参数

### 订阅话题

| 相机类型 | 话题 |
|----------|------|
| 单目 | `camera/image_raw` |
| 双目 | `camera/left/image_raw`、`camera/right/image_raw` |
| RGB-D | `camera/color/image_raw`、`camera/depth/image_raw` |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/camera_pose` | `nav_msgs/Odometry` | 当前相机位姿 |
| `~/keyframes` | `geometry_msgs/PoseArray` | 地图关键帧 |
| `~/keyframes_2d` | `geometry_msgs/PoseArray` | XY 平面投影关键帧 |

### 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `odom_frame` | `odom` | 里程计坐标系 |
| `map_frame` | `map` | 地图坐标系 |
| `robot_base_frame` | `base_link` | 机器人基座坐标系 |
| `camera_frame` | `camera_frame` | 相机坐标系 |
| `publish_tf` | `true` | 是否发布 `map` → `odom` TF |
| `publish_keyframes` | `true` | 是否发布关键帧 |
| `odom2d` | `false` | 是否将 odom 约束在 XY 平面 |
| `transform_tolerance` | `0.5` | TF 时间戳容忍（秒） |
| `use_exact_time` | `false` | 双目/RGB-D 是否使用精确时间同步 |
| `encoding` | `""` | 图像编码（空则使用 cv_bridge 默认） |

## 命令行参数

### `run_slam`

```text
必需:
  -v, --vocab PATH        词袋文件
  -c, --config PATH       SLAM 配置 YAML

可选:
  -i, --map-db-in PATH    加载地图数据库
  -o, --map-db-out PATH   保存地图数据库
      --mask PATH         掩码图像
      --log-level LEVEL   info|warn|error（默认 info）
      --eval-log-dir DIR  输出 TUM 轨迹与跟踪耗时
      --disable-mapping   关闭建图模块
      --temporal-mapping  启用时序建图
  -r, --rectify           双目图像校正
  -h, --help              显示帮助
```

### `run_slam_offline`

在 `run_slam` 参数基础上增加：

```text
  -b, --bag PATH          rosbag2 目录
      --camera TOPIC      单目图像话题
      --left / --right    双目左右话题
      --color / --depth   RGB-D 彩色/深度话题
      --storage-id ID     存储格式（默认 sqlite3）
      --start-offset SEC  起始时间偏移（秒）
      --no-sleep          不等待真实时间，尽快回放
```

## 包结构

```text
autonomy_slam/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── mono.yaml / stereo.yaml / rgbd.yaml   # ROS 节点参数
│   └── atlas/mono.yaml / stereo.yaml / rgbd.yaml  # atlas SLAM 配置
├── launch/
│   ├── slam.launch.py        # 统一启动（支持 mono/stereo/rgbd）
│   ├── slam_mono.launch.py
│   ├── slam_stereo.launch.py
│   ├── slam_rgbd.launch.py
│   ├── aist_*.launch.py      # AIST Living Lab 示例
│   └── rviz.launch.py
├── rviz/autonomy_slam.rviz
├── vocab/                    # 放置 orb_vocab.fbow
├── data/aist_living_lab/     # 示例视频与配置
└── src/
```

## 相关包

| 包 | 路径 | 说明 |
|----|------|------|
| `autonomy_internnav` | `autonomy_planning/autonomy_internnav/` | NavDP 推理与训练 |
| `autonomy_simulator` | `autonomy_simulator/` | Gazebo / Fake / Habitat 仿真 |
| `autonomy_ros` | `autonomy_ros/` | 导航栈与任务 API |

## 参考

- [stella_vslam ROS 2 文档](https://stella-cv.readthedocs.io/en/latest/ros2_package.html)
- [autonomy atlas 模块](../../../autonomy/)
