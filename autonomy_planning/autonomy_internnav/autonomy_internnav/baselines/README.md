# NavDP Baselines

本目录整合自上游 [NavDP/baselines](https://github.com/InternRobotics/NavDP/tree/main/baselines)，与 ROS 推理节点、训练模块放在同一包内。

## 目录对照

| 上游 `NavDP/baselines/` | 本包路径 | ROS 集成 | 说明 |
|-------------------------|----------|----------|------|
| `navdp/` | `baselines/navdp/` | **已接入** | 默认视觉导航（`policy:=navdp`） |
| `logoplanner/` | `baselines/logoplanner/` | **已接入** | `policy:=logoplanner`，点目标 |
| `viplanner/` | `baselines/viplanner/` | **已接入** | `policy:=viplanner`，需 Mask2Former |
| `vint/` | `baselines/vint/` | **已接入** | `policy:=vint`，图像 / 无目标 |
| `nomad/` | `baselines/nomad/` | **已接入** | `policy:=nomad`，图像 / 无目标 |

## 使用方式

### ROS 多策略（推荐）

`internnav_node` 通过 launch 参数切换策略与目标类型：

```bash
ros2 launch autonomy_internnav internnav.launch.py policy:=navdp goal_type:=point
ros2 launch autonomy_internnav internnav.launch.py policy:=vint goal_type:=image
```

实现位于 `autonomy_internnav/policies/`（`factory.create_policy_inference`）。

### NavDP（Python API）

```python
from autonomy_internnav.baselines.navdp import NavDP_Agent, NavDP_Policy
# 或兼容旧路径
from autonomy_internnav.navdp import NavDP_Agent
```

ROS 节点通过 `autonomy_internnav.policies` 加载各 baseline，不再直接绑定 `NavDPInference`。

### 其他 baseline（评测 / HTTP Server）

各 baseline 保留 agent 与模型代码，可用于：

- NavDP IsaacSim 评测（原 `*_server.py` HTTP 接口，见各子目录）
- 与 ROS 节点相同的权重与配置路径约定（`weights/`、`config/navdp.yaml`）

启动 NavDP HTTP 服务（与上游 benchmark 一致）：

```bash
cd autonomy_internnav/baselines/navdp
python navdp_server.py --port 8888 --checkpoint /path/to/navdp.ckpt
```

## 共享组件

- **DepthAnythingV2**：统一放在 `baselines/navdp/depth_anything/`，`logoplanner` 复用该实现，避免重复拷贝。
- **训练**：NavDP 训练在 `autonomy_internnav/train/`（源自 InternNav），与 `baselines/navdp` 推理骨干对齐。

## 额外依赖

| Baseline | 额外 Python 包 |
|----------|----------------|
| navdp | `diffusers`, `torch`（见 `requirements.txt`） |
| vint / nomad | `efficientnet-pytorch` |
| nomad | `diffusion-policy`（上游 git 依赖） |
| viplanner | `mmdet`, `mmcv` |
| logoplanner | [Pi3](https://github.com/yyfz/Pi3)（需单独安装） |

## 上游来源

- NavDP: https://github.com/InternRobotics/NavDP
- InternNav 训练: https://github.com/InternRobotics/InternNav
