# autonomy_internnav

ROS 2 NavDP 视觉导航桥接包：将 RGB-D 图像、里程计与目标点转换为 `cmd_vel` 轨迹控制。

训练代码已从 [InternNav](InternNav/) 整合至 `autonomy_internnav/train/`，推理与训练共用 `navdp/` 模型骨干（DepthAnythingV2 + Diffusion Policy）。

## 目录

- [功能概览](#功能概览)
- [环境要求](#环境要求)
- [编译安装](#编译安装)
- [模型权重](#模型权重)
- [快速上手](#快速上手)
- [话题与服务](#话题与服务)
- [参数说明](#参数说明)
- [与仿真/SLAM 联用](#与仿真slam-联用)
- [训练 NavDP](#训练-navdp)
- [包结构](#包结构)

## 功能概览

| 模块 | 路径 | 说明 |
|------|------|------|
| ROS 推理节点 | `autonomy_internnav/node.py` | 订阅 RGB-D / odom / goal，发布 `cmd_vel` 与可视化 |
| NavDP 推理 | `autonomy_internnav/baselines/navdp/` | 默认策略（`NavDP_Policy` + `NavDP_Agent`） |
| 其他 baseline | `autonomy_internnav/baselines/{vint,nomad,viplanner,logoplanner}/` | 源自 NavDP/baselines，待 ROS 接入 |
| NavDP 训练 | `autonomy_internnav/train/` | `NavDPNet`、数据集、Trainer（源自 InternNav） |
| 权重解析 | `autonomy_internnav/weights.py` | 从 `weights/` 或环境变量加载 checkpoint |

上游模型来源：[NavDP/baselines](https://github.com/InternRobotics/NavDP/tree/main/baselines)。详见 [`autonomy_internnav/baselines/README.md`](autonomy_internnav/baselines/README.md)。

数据流：

```text
camera/rgb + camera/depth + odom + goal
        │
        ▼
  internnav_node (NavDP)
        │
        ├── cmd_vel           → 底盘 / 仿真器
        ├── navdp/plan        → critic 选中的单条轨迹
        ├── navdp/markers     → RViz：16 条 diffusion 候选（jet 着色）+ 选中轨迹
        └── navdp/overlay     → RGB 上投影轨迹（与 NavDP demo 一致）
```

NavDP 推理为 **两阶段**：diffusion 采样 `sample_num` 条候选轨迹 → critic 打分 → 执行最优一条。RViz **NavDP Overlay** 与 **NavDPMarkers** 分别对应官方 `project_trajectory` 的图像叠加与 BEV 轨迹可视化。

## 环境要求

- ROS 2（Humble 或更高版本）
- NVIDIA GPU + CUDA（推理默认 `cuda:0`）
- Python 3（ROS 节点使用系统 `/usr/bin/python3` 或工作空间 venv）
- 推理依赖见 `requirements.txt`；训练额外依赖见 `requirements-train.txt`

## 编译安装

```bash
cd /path/to/your_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select autonomy_internnav --symlink-install
source install/setup.bash

# 安装推理 Python 依赖
bash src/autonomy_ros/autonomy_planning/autonomy_internnav/scripts/install_deps.sh
```

若需训练，额外执行：

```bash
bash src/autonomy_ros/autonomy_planning/autonomy_internnav/scripts/install_train_deps.sh
```

若旧构建留下损坏的 install 符号链接：

```bash
rm -rf build/autonomy_internnav install/autonomy_internnav
colcon build --packages-select autonomy_internnav --symlink-install --allow-overriding autonomy_internnav
```

## 模型权重

默认 checkpoint：`navdp-cross-modal.ckpt`

1. 从 [NavDP 模型表单](https://docs.google.com/forms/d/e/1FAIpQLSdl3RvajO5AohwWZL5C0yM-gkSqrNaLGp1OzN9oF24oNLfikw/viewform?usp=dialog) 下载
2. 放入源码目录（**不**通过 colcon 安装大文件）：

```text
src/autonomy_ros/autonomy_planning/autonomy_internnav/weights/navdp-cross-modal.ckpt
```

或通过环境变量指定：

```bash
export NAVDP_CHECKPOINT=/path/to/navdp-cross-modal.ckpt
# 或指定权重目录
export AUTONOMY_INTERNNAV_WEIGHTS=/path/to/weights_dir
```

更多说明见 [weights/README.md](weights/README.md)。

## 快速上手

### 启动 NavDP 节点 + RViz

默认 **点目标**（`goal_type=point`），`stop_threshold=-3.0`（与 NavDP eval 一致），diffusion 采样 `sample_num=16`：

```bash
ros2 launch autonomy_internnav internnav.launch.py
```

在 RViz 中用 **2D Goal Pose** 向 `goal_pose` 发布目标。目标须在机器人**前方**（body 系 `x > 0`）；若在身后 NavDP 会将 `x` 裁为 0，可能出现 `cmd_vel = 0`。

### NavDP 目标类型（`goal_type`）

与 [NavDP server](https://github.com/InternRobotics/NavDP) 各 endpoint 对齐：

| `goal_type` | NavDP API | ROS 输入 |
|-------------|-----------|----------|
| `point`（默认） | `pointgoal_step` | `goal_pose`（RViz 2D Goal Pose） |
| `image` | `imagegoal_step` | `image_goal`（`sensor_msgs/Image`） |
| `pixel` | `pixelgoal_step` | `pixel_goal`（`geometry_msgs/PointStamped`，`x/y` 为图像像素坐标） |
| `point_image` | `navdp_step_ip_mixgoal` | `goal_pose` + `image_goal` |
| `nogoal` | `nogoal_step` | 无需目标 |

```bash
# 默认 point goal（stop_threshold=-3.0）
ros2 launch autonomy_internnav internnav.launch.py

# pixel goal：发布图像像素坐标
ros2 launch autonomy_internnav internnav.launch.py goal_type:=pixel
ros2 topic pub --once /pixel_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: camera_rgb_optical_frame}, point: {x: 320.0, y: 240.0}}"

# 无目标探索
ros2 launch autonomy_internnav internnav.launch.py goal_type:=nogoal

# 调整 diffusion 采样数（亦可在 config/navdp.yaml 中修改 sample_num）
ros2 launch autonomy_internnav internnav.launch.py sample_num:=32
```

### 切换导航策略（launch 参数）

| `policy` | `goal_type` | 说明 |
|----------|-------------|------|
| `navdp`（默认） | `point` / `image` / `pixel` / `point_image` / `nogoal` | NavDP 全模式 |
| `logoplanner` | `point` | LoGoPlanner |
| `viplanner` | `point` | VIPlanner（需配置 `m2f_checkpoint` / `m2f_config`） |
| `vint` | `image` | ViNT 图像目标 |
| `nomad` | `image` | NoMaD 图像目标 |
| `vint` / `nomad` | `nogoal` | 无目标探索 |

示例：

```bash
# LoGoPlanner 点目标
ros2 launch autonomy_internnav internnav.launch.py \
  policy:=logoplanner checkpoint:=logoplanner.ckpt

# ViNT 图像目标（发布 sensor_msgs/Image 到 image_goal）
ros2 launch autonomy_internnav internnav.launch.py \
  policy:=vint goal_type:=image checkpoint:=vint.pth

# NavDP 图像目标
ros2 launch autonomy_internnav internnav.launch.py \
  policy:=navdp goal_type:=image
```

常用 launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `policy` | `navdp` | `navdp` / `logoplanner` / `viplanner` / `vint` / `nomad` |
| `goal_type` | `point` | `point` / `image` / `pixel` / `point_image` / `nogoal` |
| `checkpoint` | `navdp-cross-modal.ckpt` | 权重文件名或绝对路径 |
| `device` | `cuda:0` | PyTorch 设备 |
| `config` | `config/navdp.yaml` | 节点参数文件 |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_sim_time` | `false` | 是否使用仿真时钟 |

节点还支持通过 launch 覆盖 `config/navdp.yaml` 中的任意 ROS 参数（如 `sample_num`、`stop_threshold`），例如：

```bash
ros2 launch autonomy_internnav internnav.launch.py stop_threshold:=-2.0 sample_num:=32
```

指定自定义权重：

```bash
ros2 launch autonomy_internnav internnav.launch.py \
  checkpoint:=/path/to/your_navdp.ckpt \
  device:=cuda:0
```

### 重置记忆

```bash
ros2 service call /internnav_node/reset std_srvs/srv/Trigger
```

清空 NavDP 时序记忆、停止运动并清除可视化 marker。

## 话题与服务

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `camera/rgb/image_raw` | `sensor_msgs/Image` | RGB 图像（可配置） |
| `camera/depth/image_raw` | `sensor_msgs/Image` | 深度图（可配置） |
| `camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `odom` | `nav_msgs/Odometry` | 机器人里程计 |
| `goal_pose` | `geometry_msgs/PoseStamped` | 点目标（`goal_type=point` 或 `point_image`） |
| `image_goal` | `sensor_msgs/Image` | 图像目标（`goal_type=image` 或 `point_image`） |
| `pixel_goal` | `geometry_msgs/PointStamped` | 像素目标（`goal_type=pixel`，`point.x/y` 为图像 u/v） |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `navdp/plan` | `nav_msgs/Path` | critic 选中的单条轨迹（默认 `base_link` 机体系） |
| `navdp/markers` | `visualization_msgs/MarkerArray` | diffusion 候选轨迹（jet 着色）+ 选中轨迹（机体系，随机器人运动） |
| `navdp/overlay` | `sensor_msgs/Image` | RGB 上投影的多条候选轨迹（NavDP `project_trajectory`） |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/internnav_node/reset` | `std_srvs/Trigger` | 重置 NavDP 记忆与输出 |

## 参数说明

默认参数文件：`config/navdp.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rgb_topic` / `depth_topic` | `camera/rgb/image_raw`、`camera/depth/image_raw` | 图像话题 |
| `odom_topic` | `odom` | 里程计话题 |
| `goal_topic` | `goal_pose` | 点目标（`goal_type=point` / `point_image`） |
| `image_goal_topic` | `image_goal` | 图像目标（`goal_type=image` / `point_image`） |
| `pixel_goal_topic` | `pixel_goal` | 像素目标（`goal_type=pixel`） |
| `overlay_topic` | `navdp/overlay` | RGB 轨迹叠加图 |
| `cmd_vel_topic` | `cmd_vel` | 速度输出话题 |
| `base_frame` | `base_link` | 机器人基座坐标系 |
| `map_frame` | `map` | 全局地图坐标系 |
| `markers_frame` | `''`（→ `base_link`） | marker 坐标系；机体系可避免 map 投影抖动 |
| `path_frame` | `''`（→ `base_link`） | `/navdp/plan` 坐标系 |
| `marker_lifetime_sec` | `0.5` | marker 自动过期时间（秒），替代每帧 DELETEALL |
| `image_size` | `224` | 模型输入分辨率 |
| `memory_size` | `8` | 历史帧数量（填满后推理更稳定） |
| `predict_size` | `24` | 预测轨迹步数 |
| `stop_threshold` | `-3.0` | critic 低于此值时停车（NavDP eval 默认；teleop 常用 `-2.0`） |
| `sample_num` | `16` | diffusion 采样条数 |
| `inference_rate_hz` | `5.0` | 推理频率 |
| `linear_speed` / `angular_speed` | `0.3` / `0.5` | 速度缩放 |
| `lookahead_index` | `3` | 轨迹跟踪前瞻点索引 |
| `depth_encoding` | `32fc1` | `32fc1`（米）、`16uc1`（毫米）、`16uc1_10k`（NavDP server `/10000`） |

## 与仿真/SLAM 联用

### Habitat 仿真（话题默认匹配）

`autonomy_simulator` Habitat 桥接发布的话题与 `navdp.yaml` 默认一致：

```bash
# 终端 1
ros2 launch autonomy_simulator simulator.launch.py sim_mode:=habitat

# 终端 2
ros2 launch autonomy_internnav internnav.launch.py use_sim_time:=false
```

### Gazebo / Fake 机器人

需确保相机与 `odom` 话题与 `navdp.yaml` 一致，或通过 launch 重映射。可与 [`autonomy_ros`](../../autonomy_ros/) 导航栈配合使用。

### 视觉 SLAM

[`autonomy_slam`](../../autonomy_slam/) 可提供定位与地图；InternNav 负责视觉点目标导航。若 SLAM 仅发布 `camera_pose` 而无 `/odom`，需 relay 或修改 `odom_topic`。详见 `autonomy_slam/README.md`。

## 训练

训练入口与 ROS 推理的 `policy` 参数对齐，统一使用 `--model` 选择模型。

| `--model` | 本包内训练 | ROS `policy` | 说明 |
|-----------|------------|--------------|------|
| `navdp`（默认） | **支持** | `navdp` | InternNav LeRobot 数据集 + NavDPNet |
| `logoplanner` | 暂不支持 | `logoplanner` | 使用 [HuggingFace 权重](https://huggingface.co/InternRobotics/LoGoPlanner) |
| `viplanner` | 暂不支持 | `viplanner` | 在上游 [viplanner](https://github.com/leggedrobotics/viplanner) 训练 |
| `vint` | 暂不支持 | `vint` | 在 [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) 训练 |
| `nomad` | 暂不支持 | `nomad` | 在 [nomad](https://github.com/robodhruv/nomad) 训练 |

NavDP 与各 baseline 的推理代码在 `baselines/`；**仅 NavDP 的训练流程已从 InternNav 迁入本包**。其余 baseline 在 NavDP 评测中直接使用上游预训练权重，本地训练需按上表到对应仓库完成，再将 checkpoint 放入 `weights/` 用于 `ros2 launch`。

### 安装训练依赖

```bash
bash scripts/install_train_deps.sh
```

### 启动训练

```bash
# NavDP 单卡（默认）
bash scripts/train/start_train.sh --name my_navdp_run --gpus 1

# 显式指定模型（与 launch policy 一致）
bash scripts/train/start_train.sh --model navdp --name my_navdp_run --gpus 8 \
  --cuda-devices 0,1,2,3,4,5,6,7
```

colcon 安装后也可使用：

```bash
train_internnav --model navdp --name my_navdp_run
# train_navdp 为兼容别名
```

非本包支持的模型会打印上游训练链接并退出，例如：

```bash
train_internnav --model vint --name vint_run
```

### 配置与产出（NavDP）

- 默认超参与数据路径：`autonomy_internnav/train/configs/navdp_default.py`
- 训练 checkpoint：`checkpoints/<run_name>/ckpts/navdp.ckpt`
- 将训练权重复制到 `weights/` 后，通过 `checkpoint` launch 参数用于推理

训练使用 LeRobot 格式数据集（InternData-N1 等），详见 InternNav 文档：
[InternNav 训练指南](https://internrobotics.github.io/user_guide/internnav/quick_start/train_eval.html)

## 包结构

```text
autonomy_internnav/
├── README.md
├── package.xml
├── setup.py
├── requirements.txt              # 推理依赖
├── requirements-train.txt        # 训练依赖
├── config/navdp.yaml             # ROS 节点默认参数
├── launch/
│   ├── internnav.launch.py       # 主启动文件
│   └── rviz.launch.py
├── rviz/navdp.rviz
├── weights/                      # checkpoint（gitignore，见 weights/README.md）
├── scripts/
│   ├── install_deps.sh
│   ├── install_train_deps.sh
│   └── train/
│       ├── start_train.sh
│       ├── train_internnav.py
│       └── train_navdp.py          # 兼容别名
├── autonomy_internnav/
│   ├── node.py                   # ROS 2 桥接节点
│   ├── inference.py              # NavDPInference 封装
│   ├── config.py                 # 参数加载
│   ├── conversions.py            # ROS ↔ NumPy 转换
│   ├── visualization.py          # RViz marker 构建
│   ├── weights.py                # checkpoint 路径解析
│   ├── policies/                 # ROS 多策略推理封装
│   │   ├── factory.py
│   │   ├── navdp_policy.py
│   │   ├── logoplanner_policy.py
│   │   ├── viplanner_policy.py
│   │   ├── vint_policy.py
│   │   └── nomad_policy.py
│   ├── baselines/                # NavDP 上游 baseline 推理
│   │   ├── navdp/                # 默认 NavDP（colormap、policy_agent 等）
│   │   ├── vint/
│   │   ├── nomad/
│   │   ├── viplanner/
│   │   └── logoplanner/
│   ├── navdp/                    # 兼容旧 import，转发至 baselines/navdp
│   └── train/                    # 训练（registry + runners，目前实现 navdp）
│       ├── registry.py
│       ├── runners/
│       ├── navdp_model.py
│       ├── train_backbone.py
│       ├── dataset.py
│       ├── navdp_trainer.py
│       └── configs/navdp_default.py
└── InternNav/                    # 上游参考仓库（可选）
```

## 参考

- [NavDP](https://github.com/InternRobotics/NavDP) — baseline 模型与 IsaacSim 评测
- [InternNav](https://github.com/InternRobotics/InternNav) — NavDP 训练与评测框架
- NavDP 权重申请：[Google Form](https://docs.google.com/forms/d/e/1FAIpQLSdl3RvajO5AohwWZL5C0yM-gkSqrNaLGp1OzN9oF24oNLfikw/viewform?usp=dialog)
