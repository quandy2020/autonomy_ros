"""NavDP GRPO 微调入口 (LeRobot v3 数据格式).

用法:
    # 使用 LeRobot v3 格式数据集:
    python scripts/train/base_train/train_grpo_lerobot_v3.py \
        --model-name navdp_grpo \
        --name navdp_grpo_lerobot_v3 \
        --ckpt-to-load <SFT权重路径> \
        --dataset-type lerobot_v3 \
        --root-dir <LeRobotDataset v3 根目录>

    # 单机多卡:
    torchrun --nproc_per_node=4 scripts/train/base_train/train_grpo_lerobot_v3.py \
        --model-name navdp_grpo \
        --name navdp_grpo_lerobot_v3 \
        --ckpt-to-load <SFT权重路径> \
        --dataset-type lerobot_v3 \
        --root-dir <LeRobotDataset v3 根目录>
"""

import os
import sys
from typing import List, Optional

# ================================================================
# 彻底禁止 HuggingFace 联网请求
# ================================================================
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_HYDRA_FULL_ERROR", "1")

# diffusion-policy via InternNav submodule if needed

import logging
from datetime import datetime
from pathlib import Path

import torch
import torch.distributed as dist
import tyro
from pydantic import BaseModel

from autonomy_internnav.train.dataset import navdp_collate_fn
from autonomy_internnav.train.dataset_lerobot_v3 import NavDP_LerobotV3_Dataset
from autonomy_internnav.train.dataset_mixed import NavDPMixedSourceDataset
from autonomy_internnav.model_registry import get_config, get_policy
from autonomy_internnav.train.logger import MyLogger
from autonomy_internnav.train.navdp_grpo_trainer import NavDPGRPOTrainer
from autonomy_internnav.train.configs.navdp_grpo import navdp_grpo_exp_cfg


class TrainCfg(BaseModel):
    """GRPO 训练命令行配置 (支持 legacy 和 lerobot_v3)."""

    name: str = "navdp_grpo_train"
    model_name: str = "navdp_grpo"
    # --- 基础 ---
    ckpt_to_load: str = ""
    # --- 数据集类型 ---
    dataset_type: str = "lerobot_v3"  # lerobot_v3 / legacy / mixed
    # --- LeRobot v3 数据集 ---
    root_dir: str = ""  # LeRobotDataset v3 根目录 (包含 meta/, data/, videos/)
    rgb_key: str = "observation.images.rgb"
    depth_key: str = "observation.images.depth"
    pointcloud_name: str = "pointcloud.ply"  # 障碍点云文件名 (相对于 root_dir)
    # --- 旧版数据集 (兼容) ---
    dataset_navdp: str = "data/datasets/navdp_dataset_lerobot.json"
    # --- 混合数据集配置 ---
    sources: Optional[List[dict]] = None  # 混合数据源配置列表
    epoch_size: Optional[int] = None  # 每个epoch的样本数 (仅mixed模式)
    # --- 通用数据配置 ---
    preload: bool = False
    memory_size: int = 8
    predict_size: int = 24
    image_size: int = 224
    scene_scale: float = 1.0
    pixel_channel: int = 4
    dataset_sample_interval: int = 4
    # 数据清洗开关与阈值（仅 lerobot_v3 生效）
    use_data_filter: bool = True
    max_continuous_step_m: float = 0.12
    max_continuous_yaw_rad: float = 0.70
    max_filter_sample_tries: int = 64
    # 目标帧分段概率分配: 将候选目标帧范围等分为 N 段, 按此权重分配采样概率
    target_segment_weights: Optional[List[float]] = None
    # --- GRPO 超参 ---
    group_size: int = 8
    num_rollout_iters: int = 100
    rollout_batch_size: int = 8
    gradient_accumulation_steps: int = 4
    # --- Episode Buffer ---
    num_scenes_per_buffer: int = 4
    buffer_size: int = 128
    buffer_epochs: int = 4
    # --- 新版 GRPO 别名参数 ---
    num_episodes_per_round: int = 128
    trajectories_per_episode: int = 16
    train_batch_size: int = 64
    num_train_epochs_per_round: int = 2  # 每轮训练epoch数
    checkpoint_every: int = 5
    max_generations: int = 1000
    rollout_num_inference_steps: Optional[int] = None
    lr: float = 5e-5
    min_lr: float = 1e-5
    weight_decay: float = 0.0
    clip_ratio_low: float = 0.2
    clip_ratio_high: float = 0.28
    kl_beta: float = 0.0
    entropy_bonus: float = 0.0
    use_ref_policy: bool = True
    use_old_policy: bool = True
    joint_logprob: bool = False
    sample_step_subset: int = 7
    # --- clip ratio 模式 ---
    # "joint": 整条去噪链 logprob 求和后做一次 ratio + clip (当前默认)
    # "stepwise": 每个去噪步单独 ratio + clip, 最后取平均
    clip_ratio_mode: str = "joint"
    log_every: int = 1
    save_every_iters: int = 50
    # --- 奖励 ---
    collision_weight: float = -5.0
    occupancy_weight: float = -1.0
    success_weight: float = 10.0
    progress_weight: float = 3.0
    success_threshold: float = 0.3
    success_effective_threshold: Optional[float] = None
    success_effective_horizon_points: int = 24
    # --- 速度/平滑/终点朝向 共享参数 (真实速度 m/s) ---
    # 真实速度: speed = ||delta_xy|| / dt, dt = frame_dt × reward_sample_interval
    # speed_frame_dt = **数据集原始采样周期** (秒, 由数据采集帧率决定, 不是控制频率).
    # reward_sample_interval = **轨迹帧间隔** (数据集原始轨迹每隔多少帧取一个动作点, 恒为4),
    #   与 dataset_sample_interval (数据加载采样频率) 无关.
    # 默认 NavDP 30Hz 数据集 × reward_sample_interval=4 → dt ≈ 0.1333s.
    speed_frame_dt: float = 1.0 / 30.0   # 数据集原始采样周期 (秒), 30Hz=1/30
    reward_sample_interval: int = 4      # 轨迹帧间隔 (恒为4, 与数据加载采样频率无关)
    action_scale: float = 4.0            # NavDP 反归一化因子 (×4)
    # --- 目标速度奖励 (target_speed_reward) ---
    # 超速和低速均惩罚: deviation_i = k * |v_i - v_target|
    target_speed_weight: float = -1.0
    target_speed: float = 0.8
    target_speed_slope: float = 1.0
    # --- 平滑奖励 (smooth_reward) ---
    smooth_weight: float = -1.0
    smooth_reduce: str = "mean"
    # --- 终点朝向奖励 (terminal_heading_reward) ---
    terminal_heading_weight: float = -1.0
    terminal_heading_window_points: int = 4
    terminal_heading_free_angle_deg: float = 45.0
    # --- TTC (Time-To-Collision) 奖励 ---
    ttc_weight: float = -8.0
    ttc_threshold: float = 4.0          # TTC 风险时间窗口 (轨迹步数)
    ttc_radius: float = 0.35            # 最近通过距离的危险判定半径 (米)
    speed_boost_rate: float = 0.2       # 速度加权系数, 每增加0.5m/s惩罚增加的比例
    # --- 目标方向投影奖励 (goal_direction_projection_reward) ---
    goal_direction_projection_weight: float = 0.0
    # --- 冻结策略 ---
    freeze_encoder: bool = True
    freeze_dit_layers: int = 11
    freeze_critic: bool = True
    # --- Critic ---
    critic_weight: float = 0.0
    # --- 训练基础设施 ---
    num_workers: int = 8
    rollout_temperature: float = 1.0
    max_grad_norm: float = 1.0
    report_to: str = "tensorboard"
    epochs: int = 1000
    # --- 数据过滤 ---
    max_heading_angle_deg: float = 60.0  # 航向角过滤阈值(度), 0 表示不过滤
    # --- goal 类型 ---
    goal_type: str = "point"  # point / image / pixel / nogoal
    # --- 特殊轨迹 (掉头 / 大角度拐弯) 混入采样 ---
    # 启用方法: 提供离线扫描生成的索引 JSON 路径 (scripts/train/preprocess_special_trajectories.py).
    # 训练时 collect_episodes_to_buffer 会先按场景采集 normal episode, 然后按比例
    # 从已选场景内 (或全局 fallback) 注入 sharp_turn / uturn segment.
    # 比例之和不应 > 1.0; 剩余比例为 normal. 默认全部关闭 (None / 0.0).
    special_trajectory_index_path: Optional[str] = None  # None = 关闭混入
    uturn_mix_ratio: float = 0.0          # 掉头样本占比, 0~1
    sharp_turn_mix_ratio: float = 0.0     # 大角度拐弯样本占比, 0~1
    # 当所选场景内的特殊样本不足时, 是否允许跨场景 fallback (会击穿 pointcloud 缓存,
    # 但能保证混入比例稳定). False 时, 特殊样本数受限于已选场景内的可用数量.
    special_allow_cross_scene_fallback: bool = True


def _make_dir(config):
    config.checkpoint_folder = config.checkpoint_folder % config.name
    config.tensorboard_dir = config.tensorboard_dir % config.name
    config.log_dir = config.log_dir % config.name
    config.output_dir = config.output_dir % config.name
    for d in [config.checkpoint_folder, config.tensorboard_dir, config.log_dir, config.output_dir]:
        os.makedirs(d, exist_ok=True)


def _is_rank_zero():
    return not dist.is_initialized() or dist.get_rank() == 0


def _override_grpo_cfg(exp_cfg, cli_cfg):
    """将 CLI 配置覆盖到 exp_cfg.grpo."""
    grpo = exp_cfg.grpo
    grpo.group_size = cli_cfg.group_size
    grpo.num_rollout_iters = cli_cfg.max_generations
    grpo.rollout_batch_size = cli_cfg.train_batch_size
    grpo.gradient_accumulation_steps = cli_cfg.gradient_accumulation_steps
    grpo.num_scenes_per_buffer = cli_cfg.num_scenes_per_buffer
    grpo.buffer_size = cli_cfg.num_episodes_per_round
    grpo.num_episodes_per_round = cli_cfg.num_episodes_per_round  # 训练器直接使用的字段
    grpo.buffer_epochs = cli_cfg.num_train_epochs_per_round  # 别名映射（实际计算iteration数）
    grpo.num_train_epochs_per_round = cli_cfg.num_train_epochs_per_round  # 训练器直接使用的字段
    grpo.train_batch_size = cli_cfg.train_batch_size  # 训练器直接使用的字段
    grpo.lr = cli_cfg.lr
    grpo.min_lr = cli_cfg.min_lr
    grpo.weight_decay = cli_cfg.weight_decay
    grpo.clip_ratio_low = cli_cfg.clip_ratio_low
    grpo.clip_ratio_high = cli_cfg.clip_ratio_high
    grpo.kl_beta = cli_cfg.kl_beta
    grpo.entropy_bonus = cli_cfg.entropy_bonus
    grpo.use_ref_policy = cli_cfg.use_ref_policy
    grpo.use_old_policy = cli_cfg.use_old_policy
    grpo.joint_logprob = cli_cfg.joint_logprob
    grpo.clip_ratio_mode = cli_cfg.clip_ratio_mode
    grpo.log_every = cli_cfg.log_every
    grpo.save_every_iters = cli_cfg.checkpoint_every
    grpo.checkpoint_every = cli_cfg.checkpoint_every  # 训练器直接使用的字段
    grpo.max_generations = cli_cfg.max_generations  # 训练器直接使用的字段
    grpo.goal_type = cli_cfg.goal_type
    grpo.max_grad_norm = cli_cfg.max_grad_norm
    grpo.rollout_temperature = cli_cfg.rollout_temperature
    grpo.rollout_num_inference_steps = cli_cfg.rollout_num_inference_steps
    grpo.trajectories_per_episode = cli_cfg.trajectories_per_episode
    grpo.max_heading_angle_deg = cli_cfg.max_heading_angle_deg
    grpo.reward.type = "composite"
    grpo.reward.collision_weight = cli_cfg.collision_weight
    grpo.reward.occupancy_weight = cli_cfg.occupancy_weight
    grpo.reward.success_weight = cli_cfg.success_weight
    grpo.reward.progress_weight = cli_cfg.progress_weight
    grpo.reward.success_threshold = cli_cfg.success_threshold
    grpo.reward.threshold = cli_cfg.success_threshold
    grpo.reward.success_effective_threshold = cli_cfg.success_effective_threshold
    grpo.reward.success_effective_horizon_points = cli_cfg.success_effective_horizon_points
    # 速度/平滑/终点朝向: 用 reward_sample_interval 推导 dt = frame_dt × reward_sample_interval
    grpo.reward.frame_dt = cli_cfg.speed_frame_dt
    grpo.reward.sample_interval = cli_cfg.reward_sample_interval
    grpo.reward.action_scale = cli_cfg.action_scale
    # 目标速度奖励
    grpo.reward.target_speed_weight = cli_cfg.target_speed_weight
    grpo.reward.target_speed = cli_cfg.target_speed
    grpo.reward.target_speed_slope = cli_cfg.target_speed_slope
    # 平滑奖励
    grpo.reward.smooth_weight = cli_cfg.smooth_weight
    grpo.reward.smooth_reduce = cli_cfg.smooth_reduce
    # 终点朝向奖励
    grpo.reward.terminal_heading_weight = cli_cfg.terminal_heading_weight
    grpo.reward.terminal_heading_window_points = cli_cfg.terminal_heading_window_points
    grpo.reward.terminal_heading_free_angle_deg = cli_cfg.terminal_heading_free_angle_deg
    # TTC 奖励
    grpo.reward.ttc_weight = cli_cfg.ttc_weight
    grpo.reward.ttc_threshold = cli_cfg.ttc_threshold
    grpo.reward.ttc_radius = cli_cfg.ttc_radius
    grpo.reward.speed_boost_rate = cli_cfg.speed_boost_rate
    # 目标方向投影奖励
    grpo.reward.goal_direction_projection_weight = cli_cfg.goal_direction_projection_weight
    # obstacle sampler 参数（传递给 collision_reward 和 occupancy_reward）
    grpo.reward.sample_radius = 3.0  # 障碍物采样半径
    grpo.reward.obstacle_sample_n = 2048  # 障碍物采样点数
    # 冻结策略
    grpo.freeze_encoder = cli_cfg.freeze_encoder
    grpo.freeze_dit_layers = cli_cfg.freeze_dit_layers
    grpo.freeze_critic = cli_cfg.freeze_critic
    # 特殊轨迹混入
    grpo.special_trajectory_index_path = cli_cfg.special_trajectory_index_path
    grpo.uturn_mix_ratio = cli_cfg.uturn_mix_ratio
    grpo.sharp_turn_mix_ratio = cli_cfg.sharp_turn_mix_ratio
    grpo.special_allow_cross_scene_fallback = cli_cfg.special_allow_cross_scene_fallback
    # Critic
    grpo.critic_weight = cli_cfg.critic_weight
    if cli_cfg.sample_step_subset > 0:
        grpo.sample_step_subset = cli_cfg.sample_step_subset

    # IlCfg
    il = exp_cfg.il
    il.root_dir = cli_cfg.root_dir
    il.dataset_navdp = cli_cfg.dataset_navdp
    il.preload = cli_cfg.preload
    il.memory_size = cli_cfg.memory_size
    il.predict_size = cli_cfg.predict_size
    il.image_size = cli_cfg.image_size
    il.scene_scale = cli_cfg.scene_scale
    il.pixel_channel = cli_cfg.pixel_channel
    il.num_workers = cli_cfg.num_workers
    il.report_to = cli_cfg.report_to
    il.ckpt_to_load = cli_cfg.ckpt_to_load
    il.target_segment_weights = cli_cfg.target_segment_weights


def _build_dataset_lerobot_v3(cli_cfg):
    """构建 LeRobot v3 格式数据集."""
    if not cli_cfg.root_dir:
        raise ValueError(
            "使用 --dataset-type lerobot_v3 时必须指定 --root-dir "
            "(指向包含 meta/, data/, videos/ 的 LeRobotDataset v3 根目录)"
        )

    train_dataset = NavDP_LerobotV3_Dataset(
        root_dirs=cli_cfg.root_dir,
        memory_size=cli_cfg.memory_size,
        predict_size=cli_cfg.predict_size,
        batch_size=cli_cfg.train_batch_size,
        image_size=cli_cfg.image_size,
        scene_data_scale=cli_cfg.scene_scale,
        trajectory_data_scale=1.0,
        pixel_channel=cli_cfg.pixel_channel,
        action_dim=3,
        debug=False,
        preload=cli_cfg.preload,
        random_digit=False,
        prior_sample=False,
        dataset_repeat=1,
        rgb_key=cli_cfg.rgb_key,
        depth_key=cli_cfg.depth_key,
        pointcloud_name=cli_cfg.pointcloud_name,
        target_segment_weights=cli_cfg.target_segment_weights,
        sample_interval=cli_cfg.dataset_sample_interval,
        use_data_filter=cli_cfg.use_data_filter,
        max_continuous_step_m=cli_cfg.max_continuous_step_m,
        max_continuous_yaw_rad=cli_cfg.max_continuous_yaw_rad,
        max_filter_sample_tries=cli_cfg.max_filter_sample_tries,
    )
    return train_dataset


def _build_dataset_legacy(cli_cfg):
    """构建 legacy 格式数据集."""
    from autonomy_internnav.train.dataset import NavDP_Base_Datset
    
    if not cli_cfg.root_dir:
        raise ValueError(
            "使用 --dataset-type legacy 时必须指定 --root-dir"
        )
    
    train_dataset = NavDP_Base_Datset(
        root_dirs=cli_cfg.root_dir,
        preload_path=cli_cfg.dataset_navdp,
        memory_size=cli_cfg.memory_size,
        predict_size=cli_cfg.predict_size,
        batch_size=cli_cfg.train_batch_size,
        image_size=cli_cfg.image_size,
        scene_data_scale=cli_cfg.scene_scale,
        trajectory_data_scale=1.0,
        pixel_channel=cli_cfg.pixel_channel,
        action_dim=3,
        debug=False,
        preload=cli_cfg.preload,
        random_digit=False,
        prior_sample=False,
        dataset_repeat=1,
        target_segment_weights=cli_cfg.target_segment_weights,
        sample_interval=cli_cfg.dataset_sample_interval,
    )
    return train_dataset


def _build_dataset_mixed(cli_cfg):
    """构建混合数据集 (支持 legacy 和 lerobot_v3)."""
    if not cli_cfg.sources:
        raise ValueError(
            "使用 --dataset-type mixed 时必须通过 --sources 指定数据源配置"
        )
    
    # 提取通用配置
    common_kwargs = {
        "memory_size": cli_cfg.memory_size,
        "predict_size": cli_cfg.predict_size,
        "batch_size": cli_cfg.train_batch_size,
        "image_size": cli_cfg.image_size,
        "scene_data_scale": cli_cfg.scene_scale,
        "pixel_channel": cli_cfg.pixel_channel,
        "action_dim": 3,
        "debug": False,
        "preload": cli_cfg.preload,
        "random_digit": False,
        "prior_sample": False,
        "dataset_repeat": 1,
        "target_segment_weights": cli_cfg.target_segment_weights,
        "sample_interval": cli_cfg.dataset_sample_interval,
    }
    
    train_dataset = NavDPMixedSourceDataset(
        sources=cli_cfg.sources,
        epoch_size=cli_cfg.epoch_size,
        **common_kwargs
    )
    return train_dataset


def main(exp_cfg, model_class, model_config_class, cli_cfg):
    # --- 分布式初始化 ---
    local_rank = int(os.getenv("LOCAL_RANK", "0"))
    world_size = int(os.getenv("WORLD_SIZE", "1"))
    rank = int(os.getenv("RANK", "0"))
    device_id = local_rank
    torch.cuda.set_device(device_id)
    device = torch.device(f"cuda:{device_id}")

    if world_size > 1:
        try:
            dist.init_process_group(backend="nccl", init_method="env://", world_size=world_size, rank=rank)
        except Exception as e:
            print(f"Distributed init failed: {e}")
            world_size = 1

    if _is_rank_zero():
        print("=" * 50)
        print(f"GRPO Training Start (LeRobot v3)  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}")
        print(f"World size: {world_size}, Rank: {rank}")
        print(f"Dataset type: {cli_cfg.dataset_type}")
        print("=" * 50)

    # --- 覆盖配置 ---
    _override_grpo_cfg(exp_cfg, cli_cfg)
    _make_dir(exp_cfg)

    # --- 加载模型 ---
    model_cfg = model_config_class(model_cfg=exp_cfg.model_dump())
    model = model_class.from_pretrained(
        pretrained_model_name_or_path=exp_cfg.il.ckpt_to_load,
        config=model_cfg,
    )
    model.to(device)
    if world_size > 1:
        model = torch.nn.parallel.DistributedDataParallel(
            model, device_ids=[local_rank], output_device=local_rank, find_unused_parameters=True
        )

    # --- Logger ---
    if _is_rank_zero():
        train_logger_filename = os.path.join(exp_cfg.log_dir, "train_grpo_lerobot_v3.log")
        train_logger = MyLogger(
            name="train_grpo_lerobot_v3",
            level=logging.INFO,
            format_str="%(asctime)-15s %(message)s",
            filename=train_logger_filename,
        )
        transformers_logger = logging.getLogger("transformers")
        if transformers_logger.hasHandlers():
            transformers_logger.handlers = []
        transformers_logger.addHandler(train_logger.handlers[0])
        transformers_logger.setLevel(logging.INFO)
        print(f"[Rank 0] Log: {train_logger_filename}")

    # --- Dataset ---
    if cli_cfg.dataset_type == "lerobot_v3":
        train_dataset = _build_dataset_lerobot_v3(cli_cfg)
    elif cli_cfg.dataset_type == "legacy":
        train_dataset = _build_dataset_legacy(cli_cfg)
    elif cli_cfg.dataset_type == "mixed":
        train_dataset = _build_dataset_mixed(cli_cfg)
    else:
        raise ValueError(
            f"不支持的数据集类型: {cli_cfg.dataset_type}, "
            f"请使用 --dataset-type lerobot_v3 / legacy / mixed"
        )

    if _is_rank_zero():
        print(f"[Rank 0] Dataset loaded: {len(train_dataset)} episodes")
        print(f"[Rank 0] Dataset type: {cli_cfg.dataset_type}")

    # --- Trainer ---
    trainer = NavDPGRPOTrainer(
        config=exp_cfg,
        model=model,
        train_dataset=train_dataset,
        data_collator=navdp_collate_fn,
    )

    # --- 创建优化器和学习率调度器 ---
    # 计算正确的总步数: max_generations * iters_per_round / gradient_accumulation_steps
    # 其中 iters_per_round = num_train_epochs_per_round * (num_episodes * trajectories // batch_size)
    grpo = exp_cfg.grpo
    num_iters_per_epoch = grpo.num_episodes_per_round * grpo.trajectories_per_episode // grpo.train_batch_size
    num_train_iters_per_round = grpo.num_train_epochs_per_round * num_iters_per_epoch
    num_training_steps = grpo.num_rollout_iters * num_train_iters_per_round // grpo.gradient_accumulation_steps
    trainer.create_optimizer_and_scheduler(num_training_steps)

    # --- 开始训练 ---
    trainer.train()

    if _is_rank_zero():
        for handler in train_logger.handlers:
            handler.flush()

    if world_size > 1:
        dist.destroy_process_group()


if __name__ == "__main__":
    cli_cfg = tyro.cli(TrainCfg)

    print("\n" + "=" * 50)
    print("GRPO FINE-TUNING CONFIGURATION (LeRobot v3):")
    print("=" * 50)
    for key, value in vars(cli_cfg).items():
        print(f"  {key}: {value}")
    print("=" * 50 + "\n")

    exp_cfg = navdp_grpo_exp_cfg
    exp_cfg.name = cli_cfg.name
    exp_cfg.num_gpus = len(exp_cfg.torch_gpu_ids)

    model_class, model_config_class = get_policy("NavDPGRPO_Policy"), get_config("NavDPGRPO_Policy")

    main(exp_cfg, model_class, model_config_class, cli_cfg)
