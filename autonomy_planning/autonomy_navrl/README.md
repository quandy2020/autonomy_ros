# autonomy_navrl

Isaac Lab 模块化 RL 训练框架：多机器人形态、可插拔任务/奖励/运控，训练与 ROS 2 部署解耦。

**文档**：[架构](docs/ARCHITECTURE.md) · [包结构](docs/STRUCTURE.md) · [工程规范](docs/ENGINEERING.md) · [Go2W](docs/GO2W.md) · [扩展插件](docs/EXTENDING.md) · [配置参考](config/reference.yaml)

## 快速开始

```bash
# Docker 内（/workspace/autonomy）
source /opt/ros/humble/setup.bash
colcon build --packages-select autonomy_navrl --symlink-install
source install/setup.bash
cd src/autonomy_ros/autonomy_planning/autonomy_navrl

export OMNI_KIT_ACCEPT_EULA=YES
bash scripts/navrl.sh validate go2w
bash scripts/navrl.sh setup go2w --fetch
bash scripts/navrl.sh train go2w-jit -n 16
```

```python
from autonomy_navrl import NavrlFramework

fw = NavrlFramework.from_yaml('config/go2w.yaml', profile='jit')
env = fw.create_env(backend='isaac')
```

## 环境要求

| 场景 | 要求 |
|------|------|
| 训练 | NVIDIA GPU + Isaac Lab Docker（`nvcr.io/nvidia/isaac-lab:2.2.0`） |
| 部署 | ROS 2 Humble+、PyTorch |
| Isaac 冒烟 | ≥16 GB 可用内存；驱动建议 R580（595+ 可能 segfault） |

Mock 后端（`smoke mock`）仅验证 PPO 管线，**不训练视觉策略**。

## 安装

```bash
colcon build --packages-select autonomy_navrl --symlink-install
source install/setup.bash
bash scripts/navrl.sh deps train    # Isaac 容器内
bash scripts/navrl.sh deps deploy   # ROS 部署侧
```

## 配置

```text
config/
  deploy.yaml       # ROS 部署
  reference.yaml    # 全字段参考
  go2w.yaml         # profiles: jit | kinematic | demo | webrtc
  s10.yaml          # profiles: default | jdrobot | webrtc | 100envs | smoke | mock
```

Preset 映射到 `config:profile`，也可显式指定 `-c` / `-p`。完整字段见 `config/reference.yaml`。

## CLI（`scripts/navrl.sh`）

唯一入口。子命令 + preset + 全局短选项，其余参数透传给 `train_navrl` / demo。

### 子命令

| 命令 | 说明 |
|------|------|
| `train [preset]` | PPO 训练（preset 默认 `s10`） |
| `demo` | Go2W 精准对齐 demo |
| `setup go2w` | URDF / JIT 权重（`--fetch` \| `--urdf` \| `--policy`） |
| `validate [go2w]` | 配置与插件校验 |
| `smoke mock\|isaac\|webrtc` | 冒烟测试 |
| `tensorboard -r RUN` | TensorBoard |
| `deps train\|deploy` | Python 依赖 |
| `help presets\|webrtc\|memory` | 帮助 |

### 全局选项（train / demo / smoke）

| 选项 | 说明 |
|------|------|
| `-c, --config PATH` | 配置文件 |
| `-p, --profile NAME` | profile 名 |
| `-b, --backend isaac\|mock` | 仿真后端 |
| `-n, --num-envs N` | 并行环境数 |
| `-w, --webrtc` | WebRTC 推流（livestream=2，自动切 webrtc profile） |
| `--livestream 0\|1\|2` | 显式推流模式（1=公网，2=局域网） |
| `--public-ip IP` | 公网推流地址 |

Demo 额外选项：`-x` / `-y` 目标坐标，`--yaw` 目标航向（度）。

### 常用示例

```bash
bash scripts/navrl.sh help presets

# Go2W
bash scripts/navrl.sh setup go2w --fetch
bash scripts/navrl.sh train go2w-jit -n 16
bash scripts/navrl.sh train go2w              # 运动学基线
bash scripts/navrl.sh train go2w -w           # WebRTC + 位姿可视化
bash scripts/navrl.sh demo -w -x 2 --yaw 45   # demo + 推流

# S10
bash scripts/navrl.sh train s10
bash scripts/navrl.sh train jdrobot -n 64
bash scripts/navrl.sh train s10 -w
bash scripts/navrl.sh train 100envs -n 100

# 运维
bash scripts/navrl.sh validate go2w
bash scripts/navrl.sh smoke isaac
bash scripts/navrl.sh tensorboard -r navrl_go2w_jit
```

### Preset 一览

| preset | 配置 : profile | 别名 |
|--------|----------------|------|
| `go2w-jit` | `go2w.yaml` : `jit` | `jit` |
| `go2w` | `go2w.yaml` : `kinematic` | `kinematic` |
| `go2w-webrtc` / `go2w -w` | `go2w.yaml` : `webrtc` | — |
| `s10` | `s10.yaml` : `default` | `default` |
| `jdrobot` | `s10.yaml` : `jdrobot` | — |
| `webrtc` / `s10 -w` | `s10.yaml` : `webrtc` | `s10-webrtc` |
| `100envs` / `smoke` / `mock` | `s10.yaml` : 同名 | — |

也可直接调用 Python CLI：

```bash
python -m autonomy_navrl.train.cli --config config/go2w.yaml --profile jit -n 16
python -m autonomy_navrl.train.cli --list-plugins
```

Checkpoint：`checkpoints/<run_name>/ckpts/navrl_latest.pt`

## WebRTC 推流

```bash
bash scripts/navrl.sh train go2w -w          # 推荐
bash scripts/navrl.sh train s10 -w --livestream 1 --public-ip 192.168.1.10  # 公网
bash scripts/navrl.sh help webrtc            # 客户端与 Docker 端口说明
```

需安装 [NVIDIA Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html)，默认端口 `49100`。

## S10

```bash
bash scripts/navrl.sh setup s10 urdf
bash scripts/navrl.sh train-s10-loco --headless   # Isaac Lab 低层运控训练
bash scripts/navrl.sh demo-s10-loco --vx 0.3      # 运控冒烟
bash scripts/navrl.sh train s10-jit -n 16         # 上层导航（需 policy.pt）
```

详见 [docs/S10.md](docs/S10.md)。

## Go2W

| profile | 运控 | 任务 | 奖励 |
|---------|------|------|------|
| `jit` | `go2w_jit` | `go2w_precision_pose` | `precision_iou` |
| `kinematic` | `root_velocity` | `precision_pose` | `jdrobot` |
| `demo` | `go2w_jit` | 固定目标对齐 | `jdrobot` |
| `webrtc` | `root_velocity` | `precision_pose` | `jdrobot` + viz |

详见 [docs/GO2W.md](docs/GO2W.md)。

## S10 / JDRobot

```bash
bash scripts/navrl.sh train jdrobot -n 64
```

自定义 URDF：修改 YAML 中 `robot.urdf_path` 与 link 名（见 `config/reference.yaml`）。

## ROS 2 部署

```bash
export NAVRL_CHECKPOINT=/path/to/navrl_latest.pt
ros2 launch autonomy_navrl navrl.launch.py
```

订阅 RGB-D / IMU / odom / goal，发布 `cmd_vel`（或 `SkateCmd`）。参数见 `config/deploy.yaml`。

## 包结构

```text
autonomy_navrl/
├── config/                 # YAML 配置
├── scripts/
│   └── navrl.sh            # 唯一 Shell 入口
├── docs/                   # 架构 / 工程规范 / Go2W / 扩展
└── autonomy_navrl/
    ├── core/               # spec、registry、types、YAML、几何、采样
    ├── plugins/            # 任务、奖励、运控、go2w、jdrobot
    ├── models/             # 编码器 + 策略 + factory
    ├── algorithms/         # PPO + factory
    ├── preprocessing/      # numpy RGBD/IMU/odom（deploy/mock）
    ├── env/isaac/          # Isaac Lab DirectRLEnv
    ├── train/              # CLI
    ├── deploy/             # ROS 2 推理
    └── viz/                # Isaac 位姿 / RViz
```

扩展新机器人或任务：[docs/EXTENDING.md](docs/EXTENDING.md) · 工程规范：[docs/ENGINEERING.md](docs/ENGINEERING.md)。
