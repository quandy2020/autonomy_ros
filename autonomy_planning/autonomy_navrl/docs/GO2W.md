# Go2W 完整指南

Unitree Go2W 四足轮足在 `autonomy_navrl` 中的完整适配（对齐 Isaac Lab `unitree_go2w`）。

## 一键命令（`scripts/navrl.sh`）

```bash
bash scripts/navrl.sh setup go2w --fetch   # URDF + JIT 权重
bash scripts/navrl.sh validate go2w
bash scripts/navrl.sh train go2w-jit -n 16
bash scripts/navrl.sh train go2w -w        # WebRTC 推流
bash scripts/navrl.sh demo -w
bash scripts/navrl.sh tensorboard -r navrl_go2w_jit
```

## 能力清单

| 模块 | 说明 |
|------|------|
| URDF / Mesh | `navrl.sh setup go2w urdf` |
| JIT 运控 | `locomotion.backend: go2w_jit` |
| Bbox IoU + 连续成功 | `task.consecutive_iou` |
| precision_iou 奖励 | `use_bbox_iou: true` |
| RGB-D + IMU + Odom | 96×72，`radar` / `camera_optical_frame` |

## 配置

`config/go2w.yaml` 通过 **profile** 选择场景（默认 `jit`）：

| profile | 用途 |
|---------|------|
| `jit` | **推荐训练**（JIT 运控 + precision_iou） |
| `kinematic` | root_velocity 基线 |
| `demo` | 精准对齐 demo |
| `webrtc` | WebRTC 可视化调试 |

## 与 Isaac Lab 差距

| 已有 | 未有 |
|------|------|
| JIT + bbox IoU + consecutive success | 三阶段 curriculum |
| DirectRLEnv + PPO | RSL-RL / ManagerBased |

详见 [ARCHITECTURE.md](ARCHITECTURE.md)。
