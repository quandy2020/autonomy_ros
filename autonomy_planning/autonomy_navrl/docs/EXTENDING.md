# 扩展指南

本文说明如何在不修改 core 的前提下添加自定义插件。

## 1. 自定义任务

在任意模块中（建议 `my_pkg/my_task.py`）：

```python
import torch
from autonomy_navrl.plugins.tasks import BaseTask, task_registry

@task_registry.register('orbit')
class OrbitTask(BaseTask):
    name = 'orbit'

    def reset_goals(self, env_ids: torch.Tensor) -> None:
        radius = float(self.params.get('radius', 2.0))
        self.env._goals[env_ids, 0] = radius
        self.env._goals[env_ids, 1] = 0.0
        self.env._goal_yaws[env_ids] = 0.0

    def goal_state_features(self, pos_xy, yaw):
        # 返回 (N, K) 目标相关特征
        ...

    def is_success(self, distance, yaw_error):
        return distance < self.env.cfg.goal_tolerance_m
```

确保训练入口 **import 该模块**（注册副作用），然后在 YAML 中：

```yaml
task:
  kind: orbit
  radius: 3.0
```

## 2. 自定义动作模型

```python
from autonomy_navrl.plugins.actions import BaseActionAdapter, action_registry

@action_registry.register('skid_steer')
class SkidSteerAction(BaseActionAdapter):
    def pre_physics_step(self, actions):
        ...

    def root_velocity_world(self, yaw):
        ...
```

```yaml
control:
  action_model: skid_steer
  action_dim: 2
```

## 3. 自定义奖励

```python
from autonomy_navrl.plugins.rewards import BaseRewardComputer, RewardStepContext, reward_registry

@reward_registry.register('sparse')
class SparseRewardComputer(BaseRewardComputer):
    ...
```

```yaml
reward:
  profile: sparse
```

**内置示例奖励**（见 `plugins/rewards_extras.py`）：

| profile | 说明 |
|---------|------|
| `sparse` | 仅成功 +1 / 碰撞 -1 |
| `precision_iou` | 位姿 IoU 奖励 + 成功 bonus（对齐 Isaac Lab Go2W） |

```yaml
reward:
  profile: precision_iou
  precision_iou:
    iou_threshold: 0.95
    success_bonus: 50.0
    position_tolerance_m: 0.08
    yaw_tolerance_rad: 0.12
```

## 4. 分层 Locomotion（JIT）

高层导航策略输出 `(vx, vy, w)`，低层 JIT 策略输出关节目标。

```yaml
locomotion:
  backend: go2w_jit          # 或 root_velocity（默认运动学）
  decimation: 4
  policy_path: weights/go2w/policy.pt
  scale: [0.5, 0.5, 1.0]
  clip: {max_vx: 0.5, max_vy: 0.5, max_w: 1.0}
```

准备 Go2W JIT 权重：

```bash
bash scripts/navrl.sh setup go2w urdf
bash scripts/navrl.sh setup go2w policy
bash scripts/navrl.sh train go2w-jit
```

自定义 locomotion 后端：

```python
from autonomy_navrl.plugins.locomotion.base import BaseLocomotionController, locomotion_registry

@locomotion_registry.register('my_locomotion')
class MyLocomotion(BaseLocomotionController):
    def reset(self, env_ids): ...
    def apply(self): ...
```

## 5. 自定义可视化

在 `viz/isaac/modules.py` 中添加：

```python
class MyMarkerModule(PoseVizModule):
    name = 'my_marker'
    ...
MODULE_CLASSES['my_marker'] = MyMarkerModule
```

## 6. 新机器人 preset

在 `plugins/robots.py`：

```python
ROBOT_PRESETS['my_robot'] = {
    'kind': 'quadruped',
    'base_link': 'trunk',
    'spawn_position': [0, 0, 0.55],
}
```

```yaml
robot:
  preset: my_robot
  urdf_path: /path/to.urdf
```

## 7. 验证扩展

```bash
train_navrl --list-plugins          # 确认已注册
train_navrl --config my.yaml --dry-run
```

## 内置示例：waypoint_nav

顺序访问多个路标点，字段见 `config/reference.yaml`（`task.kind: waypoint_nav`）：

```yaml
task:
  kind: waypoint_nav
  waypoints:
    - [3.0, 0.0, 0.0]
    - [3.0, 2.0, 1.57]
    - [0.0, 2.0, 3.14]
```

中间路标到达后自动切换目标，仅最后一个路标触发 episode 终止。
