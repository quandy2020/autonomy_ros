# autonomy_navrl 包结构（v0.4）

## 分层

```text
autonomy_navrl/
├── core/                 # 框架 API、配置 spec、Registry、共享工具
│   ├── api.py            NavrlFramework
│   ├── spec.py           FrameworkConfig
│   ├── config_loader.py  YAML _base + profiles
│   ├── types.py          DeployConfig, VelocityCommand, …
│   ├── geometry.py       body-frame 位姿特征
│   ├── sampling.py       目标/障碍物采样
│   ├── state.py          本体感知状态向量
│   └── registry.py       Registry[T]
├── plugins/              # Isaac MDP 插件
│   ├── bootstrap.py      ensure_plugins()
│   ├── isaac_sensors.py  Isaac prim + torch 观测
│   ├── go2w/  jdrobot/
├── env/                  # 训练环境
│   ├── mock_env.py  mock_reward.py
│   └── isaac/
├── models/               # 神经网络
├── algorithms/           # PPO + factory
├── preprocessing/        # numpy RGBD/IMU/odom（deploy / mock）
├── control/commands.py   # 速度指令 scale/clip
├── train/  deploy/  viz/  demo/
```

## 命名规范

| 规则 | 示例 |
|------|------|
| 工厂 | `env/factory`, `models/factory`, `algorithms/factory` |
| 注册 | `ensure_plugins()` |
| Isaac 专用 | `isaac_sensors`, `isaac_env` |
| Mock 专用 | `mock_env`, `mock_reward` |
| 类名 | `Navrl*`（无 Quadruped 别名） |

## 依赖方向

```text
YAML → core → plugins → env/isaac → models → algorithms → train | deploy
                              ↓
                        preprocessing（deploy/mock 观测）
```

## v0.4 破坏性变更

| 移除 | 替代 |
|------|------|
| `common/` | `core/`（types, config_loader, geometry, sampling, state） |
| `sensors/` | `preprocessing/` |
| `Quadruped*` 全部别名 | `Navrl*` |
| `PpoTrainer` | `PpoAlgorithm` |
| `register_builtin_plugins()` | `ensure_plugins()` |
