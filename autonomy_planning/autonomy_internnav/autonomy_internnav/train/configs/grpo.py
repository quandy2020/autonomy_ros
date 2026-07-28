"""GRPO 算法相关配置 (pydantic).

设计原则: 字段命名与 RLinf YAML 中 ``algorithm.*`` 节保持一致, 便于跨项目迁移.
"""

from typing import Any, Dict, Optional

from pydantic import BaseModel, Field


class RewardCfg(BaseModel, extra="allow"):
    """奖励函数配置. 通过 ``type`` 字段在 build_reward_fn 中派发."""

    type: str = "composite"  # "dummy" / "collision" / "composite" / ...
    # dummy 模式参数
    mode: Optional[str] = "zero"  # dummy 模式: zero/constant/random
    value: Optional[float] = 0.0
    random_std: Optional[float] = 1.0
    # collision 模式参数
    # collision_threshold = 膨胀半径 - 机器人半径 ≈ 0.30 - 0.25 = 0.05m
    # 当质点到膨胀障碍物距离 < collision_threshold 时视为碰撞
    collision_threshold: Optional[float] = 0.05  # 碰撞判定阈值 (米)
    collide_value: Optional[float] = 1.0
    safe_value: Optional[float] = 0.0
    # composite 模式: 分项奖励权重
    collision_weight: float = -5.0
    occupancy_weight: float = -1.0
    success_weight: float = 10.0
    progress_weight: float = 3.0
    success_threshold: float = 0.3
    success_effective_threshold: Optional[float] = None
    success_effective_horizon_points: int = 24

    # ----- 速度/平滑/终点朝向 共享参数 -----
    # 真实速度计算: speed = ||delta_xy|| / dt, dt = frame_dt * sample_interval
    # frame_dt = **数据集原始采样周期** (秒), 默认 1/30 ≈ 0.0333 (30Hz 数据集),
    # 注意是数据采集时的帧率, 不是模型部署时的控制频率.
    # 例: 30Hz 数据 + sample_interval=4 → dt ≈ 0.133s.
    frame_dt: float = 1.0 / 30.0      # 数据集原始采样周期 (秒), 30Hz=1/30
    sample_interval: int = 4          # 与数据集 sample_interval 保持一致 (训练入口自动同步)
    action_scale: float = 4.0         # 动作反归一化因子 (NavDP 约定 ×4)

    # ----- legacy: 限速软惩罚 (speed_limit_reward, 兼容旧 pipeline) -----
    # 默认权重 0 表示关闭, 推荐使用 target_speed.
    speed_limit_weight: float = 0.0
    speed_limit: float = 1.0          # 真实速度上限 (m/s, 仅 legacy 使用)
    base_penalty_ratio: float = 1.0   # 超速起步惩罚倍数 (单点 1/predict_size)
    step_penalty_ratio: float = 12.0  # 每超 1m/s 增量倍数 (单点 12/predict_size)

    # ----- 目标速度奖励 (target_speed_reward) -----
    # 围绕 v_target 的对称偏离量: deviation = sum_i k * |v_i - v_target|
    # 超速和低速均惩罚, 输出 >= 0, 越大越差; 因此权重应为负.
    target_speed_weight: float = -1.0
    target_speed: float = 0.8         # 目标真实速度 (m/s)
    target_speed_slope: float = 1.0   # 单点偏离斜率 k

    # ----- 平滑奖励 (smooth_reward) -----
    # 步长均值: metric = mean(d_i), 用于控制加速度
    # 输出 >= 0, 越大越差; 因此权重应为负.
    smooth_weight: float = -1.0
    smooth_reduce: str = "mean"          # 'mean' 或 'sum'

    # ----- 终点朝向奖励 (terminal_heading_reward) -----
    # 输出 0~1 的偏离量, 仅在末端落在起点-目标连线段且未到达时生效;
    # 权重应为负.
    terminal_heading_weight: float = -1.0
    terminal_heading_window_points: int = 4   # heading = p[-1] - p[-K]
    terminal_heading_free_angle_deg: float = 45.0
    # success_threshold 由 SuccessRewardFn / ProgressRewardFn / TerminalHeadingRewardFn 共用.

    # ----- TTC (Time-To-Collision) 奖励 -----
    # 基于物理意义的TTC计算: 假设机器人保持当前运动趋势, 最快还有多少秒碰撞.
    # 机器人速度从actions推导真实物理速度 (m/s), 航向角从位移方向推导.
    # 障碍物假设为静止 (数据集中无障碍物速度信息).
    # 输出 >= 0, 越大代表碰撞风险越高; 权重应为负.
    ttc_weight: float = -8.0
    ttc_threshold: float = 2.0    # TTC 风险时间窗口 (秒)
    ttc_radius: float = 0.35      # 最近通过距离的危险判定半径 (米)
    # 渐进式惩罚参数
    speed_boost_rate: float = 0.2     # 速度加权系数, 每增加0.5m/s惩罚增加的比例

    # ----- 目标方向投影奖励 (goal_direction_projection_reward) -----
    # 把轨迹终点投影到「起点→目标」方向, 只奖励正投影.
    # 输出 >= 0, 投影越大越好; 配合**正权重**使用.
    goal_direction_projection_weight: float = 0.0  # 默认关闭, 设正值启用


class GrpoCfg(BaseModel, extra="allow"):
    """GRPO 微调超参.

    与 ``IlCfg`` 平级, 训练入口根据 ``ExpCfg.grpo`` 是否存在选择 GRPO trainer.
    """

    # --- 算法注册表派发 ---
    adv_type: str = "grpo"
    loss_type: str = "actor"

    # --- 组内归一化 ---
    group_size: int = 8
    normalize_advantages: bool = True

    # --- PPO clip ---
    clip_ratio_low: float = 0.2
    clip_ratio_high: float = 0.28

    # --- KL 与熵 (可选) ---
    kl_beta: float = 0.0
    entropy_bonus: float = 0.0
    use_ref_policy: bool = True  # 是否使用参考策略（π_ref）用于 KL-loss
    use_old_policy: bool = True  # 是否使用旧策略（π_old）用于 ratio 计算

    # --- 优势过滤 (可选) ---
    filter_rewards: bool = False
    rewards_lower_bound: float = -1e9
    rewards_upper_bound: float = 1e9

    # --- 训练循环 ---
    num_inner_epochs: int = 1  # 每个 rollout 后, 在同一份 chain 上做几次梯度更新
    rollout_batch_size: int = 4  # 每次 rollout 从 buffer 中抽取的 episode 数 (实际 batch=rollout_batch_size*group_size)
    num_rollout_iters: int = 100  # 总 rollout 迭代数 (替代 IlCfg.epochs)
    gradient_accumulation_steps: int = 4  # 跨 iteration 梯度累积; 等效 batch = rollout_batch_size * group_size * 此值
    rollout_num_inference_steps: Optional[int] = None  # None -> 用 scheduler.num_train_timesteps
    rollout_temperature: float = 1.0  # >1.0 放大后验方差σₜ×T提高探索度, <1.0降低随机性; log-prob始终用原始std

    # --- Episode Buffer (场景级缓存) ---
    num_scenes_per_buffer: int = 4  # 每次 buffer 采集选取的场景数量
    buffer_size: int = 128  # 每次 buffer 缓存的 episode 总数 (从选中场景内随机采集)
    buffer_epochs: int = 4  # 对每个 buffer 训练多少个 epoch 后换场景重新采集
    # 新版配置兼容字段
    num_episodes_per_round: int = 128  # 等同于 buffer_size
    trajectories_per_episode: int = 16  # 每个 episode 采样的轨迹数 (等同于 group_size)
    train_batch_size: int = 64  # 训练时的 batch size
    checkpoint_every: int = 5  # 每 N 代保存 checkpoint
    max_generations: Optional[int] = None  # 最大训练代数，None 表示无限制

    # --- log-prob 聚合 ---
    joint_logprob: bool = False  # True: 整条 chain 的 logprob 求和; False: 逐步 token-level clip (推荐)
    sample_step_subset: Optional[int] = None  # None 时训练所有去噪步; 指定整数则随机采样 K 步以省显存
    # --- clip ratio 模式 ---
    # "joint": 整条去噪链 logprob 求和后做一次 ratio + clip, 再乘 advantage 求 loss (当前默认)
    # "stepwise": 每个去噪步单独计算 ratio + clip × advantage, 最后取平均值做 loss
    clip_ratio_mode: str = "joint"

    # --- 学习率 (覆盖 IlCfg.lr) ---
    lr: Optional[float] = None
    min_lr: Optional[float] = None  # 终止学习率; None 时回退到 lr * 0.5
    weight_decay: float = 0.0
    max_grad_norm: float = 1.0

    # --- 采样 goal 类型: point / image / pixel / nogoal ---
    goal_type: str = "point"

    # --- Critic 辅助损失 (GRPO 微调默认关闭) ---
    critic_weight: float = 0.0  # 0.0=关闭 critic loss; >0 时启用 SFT critic 辅助损失
    critic_use_augment: bool = True  # True: 同时用 label+augment 两条 critic 监督; False: 仅 label

    # --- 冻结策略 ---
    freeze_encoder: bool = True  # True=冻结 rgb_model + depth_model
    freeze_dit_layers: int = 11  # 冻结 DiT decoder 的前 N 层; 0=不冻结任何层
    freeze_critic: bool = True   # True=冻结 critic_head

    # --- 奖励 ---
    reward: RewardCfg = Field(default_factory=RewardCfg)

    # --- 特殊轨迹 (掉头 / 大角度拐弯) 混入采样 ---
    # 启用方法: 提供离线扫描生成的索引 JSON 路径 (scripts/train/preprocess_special_trajectories.py).
    # 训练时 collect_episodes_to_buffer 会先按场景采集 normal episode, 然后按比例
    # 从已选场景内 (或全局 fallback) 注入 sharp_turn / uturn episode.
    # 比例之和不应 > 1.0; 剩余比例为 normal.
    special_trajectory_index_path: Optional[str] = None  # None = 关闭混入
    uturn_mix_ratio: float = 0.0          # 掉头样本占比, 0~1
    sharp_turn_mix_ratio: float = 0.0     # 大角度拐弯样本占比, 0~1
    # 当所选场景内的特殊样本不足时, 是否允许跨场景 fallback (会击穿 pointcloud 缓存,
    # 但能保证混入比例稳定). False 时, 特殊样本数受限于已选场景内的可用数量.
    special_allow_cross_scene_fallback: bool = True

    # --- 日志 ---
    log_every: int = 1
    save_every_iters: int = 50
