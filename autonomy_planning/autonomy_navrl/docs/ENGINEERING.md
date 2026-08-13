# 工程规范

`autonomy_navrl` 遵循 Google Python Style Guide 的实用子集，并与 ROS 2 / Isaac Lab 集成约束对齐。

## 模块分层

```text
autonomy_navrl/
├── core/           # 框架 API、配置 spec、Registry、types、YAML、几何、采样
├── plugins/        # 可扩展 MDP 组件（task/reward/action/locomotion）
├── env/            # 仿真环境（isaac/、mock/）
├── models/         # 多模态编码器 + 策略网络 + factory
├── algorithms/     # 训练算法（PPO / GRPO / diffusion 注册表）
├── preprocessing/  # numpy 传感器预处理（deploy / mock）
├── train/          # 训练 CLI 入口
├── deploy/         # ROS 2 部署
├── control/        # commands.py 统一速度指令
└── viz/            # Isaac / RViz 可视化
```

**依赖方向（单向）**：

```text
YAML → core → plugins → env/isaac → models → algorithms → train | deploy
```

禁止：`plugins/` 依赖 `env/`（采样、几何放在 `core/`）。  
禁止：`algorithms/` 硬编码具体 encoder 结构（经 `models.factory` 构建）。

## 命名

| 类别 | 规范 | 示例 |
|------|------|------|
| 模块 | `snake_case` | `isaac_sensors.py`, `mock_reward.py` |
| 工厂 | `factory.py` | `models/factory`, `algorithms/factory`, `env/factory` |
| 注册入口 | `ensure_plugins()` | `plugins/bootstrap.py` |
| 类 | `CapWords` | `NavrlDirectEnv`, `NavrlNode` |
| 函数/变量 | `snake_case` | `sample_goal_positions` |
| 常量 | `UPPER_SNAKE` | `ROBOT_KINDS` |
| 私有 | 前缀 `_` | `_reward_computer` |
| 缩写 | 全大写或全小写一致 | `RGBD`、`PPO`（`PpoAlgorithm`） |

统一使用 `Navrl*` / `Nav*` 命名，不保留 `Quadruped*` 历史别名。

## 插件

- 注册：``@registry.register('name')`` + `plugins/bootstrap.py` 集中 import
- 查找：``registry.create(kind)``；**不要**在 `core/spec.py` 硬编码与 registry 重复的列表
- 机器人专用逻辑：``plugins/go2w/``、``plugins/jdrobot/``，不污染通用 `tasks.py`

## 配置

- 包根 `config/`：运行时 YAML（非 Python 包）
- `core/config_loader.py`：`_base` + `profiles` 合并
- 全字段参考：`config/reference.yaml`

## 注释

- 模块：一行说明职责
- 公共函数：docstring 说明输入/输出； obvious 逻辑不注释
- 禁止：注释掉的 dead code、与代码重复的废话注释

## 日志

- 训练/CLI：逐步迁移至 `logging`（模块级 `logger = logging.getLogger(__name__)`）
- 用户可见进度：允许 `print(..., flush=True)` 于 CLI 层

## 测试

- `test/test_navrl.py`：registry、config、mock smoke
- 新插件：至少覆盖 registry 注册 + config dry-run

## 扩展清单

| 添加 | 位置 | 注册 |
|------|------|------|
| 任务 | `plugins/tasks.py` 或 `plugins/<pkg>/task.py` | `task_registry` |
| 奖励 | `plugins/rewards.py` 或 robot pack | `reward_registry` |
| 运控 | `plugins/locomotion/` | `locomotion_registry` |
| 机器人 preset | `plugins/robots.py` | `ROBOT_PRESETS` |
| 融合编码器 | `models/encoders/fusion.py` | `fusion_registry` |
| 策略头 | `models/policies/gaussian.py` | `policy_registry` |
| 训练算法 | `algorithms/ppo.py` 等 | `algorithm_registry` |

详见 [STRUCTURE.md](STRUCTURE.md)、[EXTENDING.md](EXTENDING.md)、[ARCHITECTURE.md](ARCHITECTURE.md)。
