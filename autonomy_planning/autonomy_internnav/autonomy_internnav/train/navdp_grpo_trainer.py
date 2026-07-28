"""NavDP GRPO Trainer.

完整 GRPO 训练循环:
1. Episode Buffer: 从 dataset 选取若干场景, 在场景内随机采集 episode 缓存到 buffer.
2. Rollout: 从 buffer 中抽取 batch, 按 group_size 复制, 调用 model.sample_actions_with_logprob
   得到 chains / prev_logprobs / actions. 注意: NavDP 一次前向即完成 rollout,
   无需环境多步交互.
3. Reward: reward_fn.compute(trajectories) -> [total_batch] 标量 reward.
4. Advantage: 在同观测的轨迹组内计算组内优势并归一化, 存入 buffer.
5. Update: 从 buffer 随机抽取轨迹, 直接使用已计算的 advantage 做 GRPO 裁剪更新.

训练流程 (buffer-based):
- 每次 fill buffer: 选取 num_scenes_per_buffer 个场景, 采集 buffer_size 个 episode.
- 对 buffer 训练 buffer_epochs 个 epoch 后清空, 重新采集.
- 每个 epoch 内: 从 buffer 随机抽 rollout_batch_size 个 episode 做 rollout+update.
- 同一 buffer 的多次 rollout 共享场景上下文, 优势在同观测组内归一化后存入 buffer.

设计原则:
- 与 BaseTrainer / NavDPTrainer 共享 DDP / 优化器 / checkpoint 基础设施.
- 自行管理训练循环, 不依赖 HuggingFace Trainer 的 epoch/step 语义.
- reward_fn 接口与模型/算法完全解耦.
"""

import copy
import json
import math
import os
import shutil
import time
from typing import Any, Dict, List, Optional

# ================================================================
# 彻底禁止 HuggingFace 联网请求
# ================================================================
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_HYDRA_FULL_ERROR", "1")

import numpy as np
import torch
import torch.distributed as dist
from torch.utils.tensorboard import SummaryWriter

from autonomy_internnav.algorithms import build_reward_fn
from autonomy_internnav.algorithms.advantages import compute_grpo_advantages
from autonomy_internnav.train.base_trainer import BaseTrainer


class EpisodeBuffer:
    """轨迹级回放缓冲区.

    新版 GRPO 训练流程：
    1. 数据采集: 使用冻结的 π_old 在场景中收集 episode，每个 episode 包含：
       - 1个环境观测（RGB-D + 相对目标）
       - 16条同观测生成的轨迹（原始轨迹组）
       - 组内计算的优势（在同观测的16条轨迹内归一化）
       - 旧策略对应轨迹概率
    2. 存储到回放缓冲区（优势跟随轨迹保存，不保存 reward）
    3. 训练时从缓冲区随机抽取轨迹（不按观测分组），直接使用已计算的优势

    典型用法：
        buffer = EpisodeBuffer()
        buffer.add_episode(observation, trajectories, advantages, old_logprobs, ...)
        batch = buffer.sample_trajectories(batch_size=64)
    """

    def __init__(self):
        # 存储所有轨迹，每条轨迹是一个字典
        # 包含: observation, actions, advantage, old_logprob, goal_point等
        self._trajectories: List[Dict[str, Any]] = []
        # 用于采样的随机采样器
        self._rng = np.random.default_rng()
        # 记录当前 buffer 使用的场景 ID (用于验证评估)
        self._current_scene_ids: List[str] = []
        
        # 共享数据引用（避免重复存储）
        self._shared_observations: Dict[int, Dict[str, torch.Tensor]] = {}  # episode_id -> observation
        self._shared_timesteps: Optional[torch.Tensor] = None  # 所有轨迹共享同一个 timesteps
        # 显式 episode 计数器，避免 len(self._trajectories) // num_trajs 的整除假设
        self._episode_counter: int = 0

    @property
    def size(self) -> int:
        """返回缓冲区中的轨迹总数"""
        return len(self._trajectories)

    def clear(self):
        """清空缓冲区"""
        self._trajectories.clear()
        self._current_scene_ids.clear()
        self._shared_observations.clear()
        self._shared_timesteps = None
        self._episode_counter = 0

    def add_episode(
        self,
        observation: Dict[str, torch.Tensor],
        trajectories: List[Dict[str, Any]],
        advantages: torch.Tensor,
        old_logprobs: torch.Tensor,
        chains: Optional[torch.Tensor] = None,
        timesteps: Optional[torch.Tensor] = None,
        goal_embed: Optional[torch.Tensor] = None,
        rgbd_embed: Optional[torch.Tensor] = None,
        scene_ids: Optional[List[str]] = None,
        reward_details: Optional[Dict[str, torch.Tensor]] = None,
    ):
        """添加一个 episode 的数据到缓冲区.

        Args:
            observation: 环境观测数据，包含 rgb, depth, goal_point 等
            trajectories: 16条轨迹，每条包含 actions 等信息
            advantages: 每条轨迹的组内优势 [16]（在同观测的轨迹组内归一化后计算）
            old_logprobs: 旧策略对每条轨迹每个去噪步的 logprob [16, T_used]
            chains: 扩散链 [T+1, 16, predict_size, 3]（用于 logprob 计算）
            timesteps: 时间步 [T]（用于 logprob 计算）
            goal_embed: 目标嵌入 [16, 1, token_dim]
            rgbd_embed: RGB-D 嵌入 [16, seq, token_dim]
            scene_ids: 场景ID列表（可选，用于验证）
            reward_details: 分项奖励字典，包含 collision/occupancy/success/progress 等
        """
        num_trajs = len(trajectories)
        assert advantages.shape[0] == num_trajs, f"Advantages shape {advantages.shape[0]} != num_trajs {num_trajs}"
        assert old_logprobs.shape[0] == num_trajs, f"Old logprobs shape {old_logprobs.shape[0]} != num_trajs {num_trajs}"

        # 计算 episode_id（使用显式计数器，与 num_trajs 解耦）
        episode_id = self._episode_counter
        self._episode_counter += 1
        
        # 存储共享观测数据（避免重复存储）
        self._shared_observations[episode_id] = observation
        
        # 存储共享的 timesteps（所有轨迹共享）
        if self._shared_timesteps is None and timesteps is not None:
            self._shared_timesteps = timesteps

        # 为每条轨迹创建独立的数据项
        for i in range(num_trajs):
            traj_data = {
                # Episode ID（用于引用共享观测）
                "episode_id": episode_id,
                # 轨迹数据（独立）
                "actions": trajectories[i]["actions"],
                "gt_actions": trajectories[i].get("gt_actions"),
                "obstacle_points": trajectories[i].get("obstacle_points"),
                # 优势和每步 logprob（独立）
                "advantage": advantages[i],
                "old_logprob": old_logprobs[i],
                # 扩散链数据（用于 logprob 计算）
                "chain": chains[:, i] if chains is not None else None,  # [T+1, predict_size, 3]
                "goal_embed": goal_embed[i] if goal_embed is not None else None,  # [1, token_dim]
                "rgbd_embed": rgbd_embed[i] if rgbd_embed is not None else None,  # [seq, token_dim]
            }
            # 存储分项奖励
            if reward_details is not None:
                for k, v in reward_details.items():
                    traj_data[f"reward_{k}"] = v[i] if v is not None else None
            self._trajectories.append(traj_data)

        # 更新场景ID
        if scene_ids is not None:
            self._current_scene_ids = scene_ids

    def sample_trajectories(self, batch_size: int) -> Dict[str, torch.Tensor]:
        """从缓冲区随机抽取 batch_size 条轨迹.

        注意：这些轨迹来自不同的 episode，不按原始观测分组。
        优势已在组内计算完毕，直接使用即可。

        Args:
            batch_size: 要采样的轨迹数量

        Returns:
            batch: 包含所有轨迹数据的字典，每项 shape [batch_size, ...]
                   包含: rgb, depth, actions, advantage, old_logprob, chain, timesteps 等
        """
        actual_size = min(batch_size, len(self._trajectories))
        if actual_size == 0:
            return {}

        # 随机选择轨迹索引
        indices = self._rng.choice(len(self._trajectories), size=actual_size, replace=False)

        # 收集数据
        batch = {
            "rgb": [],
            "depth": [],
            "goal_point": [],
            "goal_image": [],
            "goal_pixel": [],
            "actions": [],
            "gt_actions": [],
            "obstacle_points": [],
            "advantage": [],
            "old_logprob": [],
            "chain": [],
            "timesteps": [],
            "goal_embed": [],
            "rgbd_embed": [],
            # 分项奖励字段
            "reward_collision": [],
            "reward_occupancy": [],
            "reward_success": [],
            "reward_progress": [],
            "reward_target_speed": [],
            "reward_smooth": [],
            "reward_terminal_heading": [],
            "reward_projected_collision": [],
            "reward_projected_occupancy": [],
            "reward_ttc": [],
        }

        for idx in indices:
            traj = self._trajectories[idx]
            episode_id = traj["episode_id"]
            
            # 从共享观测中获取数据
            obs = self._shared_observations[episode_id]
            batch["rgb"].append(obs["rgb"])
            batch["depth"].append(obs["depth"])
            batch["goal_point"].append(obs.get("goal_point"))
            batch["goal_image"].append(obs.get("goal_image"))
            batch["goal_pixel"].append(obs.get("goal_pixel"))
            
            # 轨迹特定数据
            batch["actions"].append(traj["actions"])
            batch["gt_actions"].append(traj.get("gt_actions"))
            batch["obstacle_points"].append(traj.get("obstacle_points"))
            batch["advantage"].append(traj["advantage"])
            batch["old_logprob"].append(traj["old_logprob"])
            batch["chain"].append(traj["chain"])
            batch["timesteps"].append(self._shared_timesteps)  # 使用共享的 timesteps
            batch["goal_embed"].append(traj["goal_embed"])
            batch["rgbd_embed"].append(traj["rgbd_embed"])
            
            # 分项奖励
            batch["reward_collision"].append(traj.get("reward_collision"))
            batch["reward_occupancy"].append(traj.get("reward_occupancy"))
            batch["reward_success"].append(traj.get("reward_success"))
            batch["reward_progress"].append(traj.get("reward_progress"))
            batch["reward_target_speed"].append(traj.get("reward_target_speed"))
            batch["reward_smooth"].append(traj.get("reward_smooth"))
            batch["reward_terminal_heading"].append(traj.get("reward_terminal_heading"))
            batch["reward_projected_collision"].append(traj.get("reward_projected_collision"))
            batch["reward_projected_occupancy"].append(traj.get("reward_projected_occupancy"))
            batch["reward_ttc"].append(traj.get("reward_ttc"))

        # 转换为 tensor
        batch_tensor = {}
        # 分项奖励字段（可能包含 None）
        reward_fields = {
            "reward_collision", "reward_occupancy", "reward_success", "reward_progress",
            "reward_target_speed", "reward_smooth", "reward_terminal_heading",
            "reward_projected_collision", "reward_projected_occupancy",
            "reward_ttc",
        }
        
        for key, values in batch.items():
            if len(values) > 0:
                # 分项奖励字段：保持列表形式（用于统计）
                if key in reward_fields:
                    batch_tensor[key] = values
                # 检查第一个元素是否为 tensor
                elif isinstance(values[0], torch.Tensor):
                    # chain 是 [T+1, pred, 3]，需要 stack 成 [B, T+1, pred, 3]
                    # 但 compute_logprob_value 期望 [T+1, B, pred, 3]，所以后续需要转置
                    if key == "chain":
                        batch_tensor[key] = torch.stack(values, dim=0)  # [B, T+1, pred, 3]
                    elif key == "goal_embed":
                        batch_tensor[key] = torch.stack(values, dim=0)  # [B, 1, token_dim]
                    elif key == "rgbd_embed":
                        batch_tensor[key] = torch.stack(values, dim=0)  # [B, seq, token_dim]
                    else:
                        batch_tensor[key] = torch.stack(values, dim=0)
                elif values[0] is not None:
                    batch_tensor[key] = torch.tensor(values)
                else:
                    batch_tensor[key] = None

        return batch_tensor


class NavDPGRPOTrainer(BaseTrainer):
    """NavDP GRPO 微调器. 继承 BaseTrainer 以复用 DDP / 优化器 / checkpoint 逻辑."""

    def __init__(self, config, model, train_dataset, data_collator=None, **kwargs):
        super().__init__(model=model, train_dataset=train_dataset, data_collator=data_collator, **kwargs)
        self.config = config
        self.grpo_cfg = config.grpo
        self.il_cfg = config.il
        self._last_loss_metrics: Dict[str, float] = {}

        # device
        if hasattr(model, 'module'):
            self.model_device = model.module.device
        else:
            self.model_device = next(model.parameters()).device

        # reward fn
        self.reward_fn = build_reward_fn(self.grpo_cfg.reward.model_dump())

        # log
        self._global_iter = 0
        self._epoch = 0
        self._start_time = time.time()
        
        # 用于记录每个 generation 的 overall reward（用于 TensorBoard）
        self._collected_overall_reward: Optional[float] = None
        self._collected_reward_details: Dict[str, Optional[float]] = {}

        # tensorboard (仅 rank 0 写入)
        self._tb_writer = None
        if self._rank == 0:
            tb_dir = getattr(config, 'tensorboard_dir', None)
            if tb_dir:
                os.makedirs(tb_dir, exist_ok=True)
                self._tb_writer = SummaryWriter(log_dir=tb_dir, flush_secs=10)
                print(f"[Rank 0] TensorBoard logging to {tb_dir}")
            else:
                print(f"[Rank 0] WARNING: tensorboard_dir is not set, TensorBoard logging disabled!")

        # 冻结 critic head (GRPO 不需要; SFT 时 critic 做辅助监督)
        self._freeze_critic_if_needed()

        # ------------------------------------------------------------------
        # 双策略框架: π_ref (永久冻结) + π_old (每轮更新)
        # ------------------------------------------------------------------
        
        # π_ref: 初始预训练 NavDP, 全程永久冻结, 用于 KL-loss 计算
        self._ref_model = None
        if getattr(self.grpo_cfg, 'use_ref_policy', False):
            # 取出原始模型 (unwrap DDP)
            raw_model = model
            if hasattr(raw_model, 'module'):
                raw_model = raw_model.module
            # 深拷贝一份作为参考模型
            self._ref_model = copy.deepcopy(raw_model)
            # 严格冻结: eval 模式 + 所有参数 requires_grad=False
            self._ref_model.eval()
            for p in self._ref_model.parameters():
                p.requires_grad = False
            self._ref_model.to(self.model_device)
            if self._rank == 0:
                ref_params = sum(p.numel() for p in self._ref_model.parameters())
                print(f"[π_ref] 深拷贝 SFT 初始权重, 共 {ref_params:,} 参数, 全程永久冻结 (用于 KL-loss)")
        
        # π_old: 每轮迭代内固定的旧策略, 用于采样、reward、ratio 计算
        # 初始状态: 拷贝当前模型 (如果从 checkpoint 恢复, 则是 checkpoint 的权重)
        self._old_model = None
        if not getattr(self.grpo_cfg, 'use_old_policy', True):
            raise ValueError("NavDP GRPO 训练必须启用 use_old_policy, 因为采样和 ratio 计算依赖固定的 π_old。")
        if getattr(self.grpo_cfg, 'use_old_policy', True):
            # 取出原始模型 (unwrap DDP)
            raw_model = model
            if hasattr(raw_model, 'module'):
                raw_model = raw_model.module
            # 深拷贝作为旧策略
            self._old_model = copy.deepcopy(raw_model)
            # 冻结: eval 模式 + 所有参数 requires_grad=False
            self._old_model.eval()
            for p in self._old_model.parameters():
                p.requires_grad = False
            self._old_model.to(self.model_device)
            if self._rank == 0:
                old_params = sum(p.numel() for p in self._old_model.parameters())
                print(f"[π_old] 初始化为当前模型权重, 共 {old_params:,} 参数, 每轮结束后更新")

        # num_inner_epochs > 1 时警告: 使用 ref policy ratio 时, 同一 chain 上多次梯度更新
        # 会导致 π_θ 偏离 π_ref, ratio 逐渐增大, 可能造成训练不稳定
        num_inner_epochs = getattr(self.grpo_cfg, 'num_inner_epochs', 1)
        if num_inner_epochs > 1 and self._rank == 0:
            print(f"[WARNING] num_inner_epochs={num_inner_epochs} > 1: "
                  f"使用 ref policy ratio 时, 同一 chain 上多次梯度更新会导致 ratio 漂移, 建议设为 1")

        # Episode Buffer (轨迹级缓存)
        self._episode_buffer = EpisodeBuffer()

        print(f"[Rank {self._rank}] NavDPGRPOTrainer init done. device={self.model_device}")

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------
    
    def _update_old_model(self, best_checkpoint_path: Optional[str] = None):
        """更新 π_old: 用当前策略 π_θ 直接替换旧策略.
        
        每个 generation 训练完成后调用, 使 π_old = π_θ, 作为下一轮的固定旧策略.
        同一批采样数据训练过程中 π_old 绝对不能变, 否则 ratio 计算无意义.
        
        Args:
            best_checkpoint_path: 保留参数, 但新逻辑中始终传 None (直接从当前模型内存拷贝)
        """
        if self._old_model is None:
            raise RuntimeError("无法更新 π_old: _old_model 未初始化。")

        # 取出目标模型
        if best_checkpoint_path is not None:
            # 从最优 checkpoint 加载
            checkpoint_file = os.path.join(best_checkpoint_path, "pytorch_model.bin")
            if os.path.exists(checkpoint_file):
                state_dict = torch.load(checkpoint_file, map_location=self.model_device)
                self._old_model.load_state_dict(state_dict)
                self._old_model.eval()
                for p in self._old_model.parameters():
                    p.requires_grad = False
                if self._rank == 0:
                    print(f"[π_old] 从最优 checkpoint 加载: {best_checkpoint_path}")
            else:
                if self._rank == 0:
                    print(f"[π_old WARNING] Checkpoint 不存在: {checkpoint_file}, 使用当前模型")
                # Fallback: 使用当前模型
                current_model = self.model
                if hasattr(current_model, 'module'):
                    current_model = current_model.module
                with torch.no_grad():
                    self._old_model.load_state_dict(current_model.state_dict())
                self._old_model.eval()
                for p in self._old_model.parameters():
                    p.requires_grad = False
        else:
            # 使用当前模型
            current_model = self.model
            if hasattr(current_model, 'module'):
                current_model = current_model.module
            with torch.no_grad():
                self._old_model.load_state_dict(current_model.state_dict())
            self._old_model.eval()
            for p in self._old_model.parameters():
                p.requires_grad = False
            if self._rank == 0:
                print(f"[π_old] 已更新为当前模型状态, 将在下一轮作为固定旧策略")
    
    @property
    def _rank(self) -> int:
        return dist.get_rank() if dist.is_initialized() else 0

    @property
    def _world_size(self) -> int:
        return dist.get_world_size() if dist.is_initialized() else 1

    def _freeze_critic_if_needed(self):
        """GRPO 微调冻结策略.

        可训练 (仅 2 类):
            - DiT decoder 后 5 层 (decoder.layers[11:16], 层内 layernorm/嵌入随层自动训练)
            - action_head (最终线性输出层)
        必须冻结 (所有其他模块):
            - 视觉编码器: rgb_model + depth_model
            - rgbd fusion: former_net + former_query + former_pe + project_layer
            - 底层编码: input_embed + cond_pos_embed
            - 所有 goal encoder: point_encoder / image_encoder / pixel_encoder
            - DiT decoder 前 11 层
            - decoder 输入侧: out_pos_embed
            - decoder 条件侧: time_emb
            - decoder 输出侧: layernorm (顶层独立 LayerNorm)
            - critic_head
            - SFT 辅助头: pixel_aux_head / image_aux_head
        """
        model = self.model
        if hasattr(model, 'module'):
            model = model.module

        frozen_count = 0
        trainable_count = 0

        def _freeze(module, name=""):
            nonlocal frozen_count
            for p in module.parameters():
                p.requires_grad = False
                frozen_count += p.numel()
            module.eval()

        # 1. 冻结视觉编码器
        if getattr(self.grpo_cfg, 'freeze_encoder', True):
            _freeze(model.rgbd_encoder.rgb_model, "rgb_model")
            _freeze(model.rgbd_encoder.depth_model, "depth_model")
            if self._rank == 0:
                print("[Freeze] Encoders frozen (rgb_model + depth_model)")

        # 2. 冻结 rgbd fusion + project_layer
        _freeze(model.rgbd_encoder.former_net, "former_net")
        _freeze(model.rgbd_encoder.former_query, "former_query")
        _freeze(model.rgbd_encoder.former_pe, "former_pe")
        _freeze(model.rgbd_encoder.project_layer, "project_layer")
        if self._rank == 0:
            print("[Freeze] rgbd fusion frozen (former_net + former_query + former_pe + project_layer)")

        # 3. 冻结底层编码: input_embed + cond_pos_embed
        _freeze(model.input_embed, "input_embed")
        _freeze(model.cond_pos_embed, "cond_pos_embed")
        if self._rank == 0:
            print("[Freeze] input_embed + cond_pos_embed frozen")

        # 4. 冻结所有 goal encoder
        _freeze(model.point_encoder, "point_encoder")
        _freeze(model.image_encoder, "image_encoder")
        _freeze(model.pixel_encoder, "pixel_encoder")
        if self._rank == 0:
            print("[Freeze] Goal encoders frozen (point + image + pixel)")

        # 5. 冻结 DiT decoder 前 N 层
        freeze_dit = getattr(self.grpo_cfg, 'freeze_dit_layers', 0)
        if freeze_dit > 0:
            num_layers = len(model.decoder.layers)
            freeze_up_to = min(freeze_dit, num_layers)
            for i in range(freeze_up_to):
                _freeze(model.decoder.layers[i], f"decoder.layers[{i}]")
            if self._rank == 0:
                print(f"[Freeze] DiT decoder layers 0-{freeze_up_to - 1} frozen ({freeze_up_to}/{num_layers} layers)")

        # 6. 冻结 decoder 输入侧: out_pos_embed
        _freeze(model.out_pos_embed, "out_pos_embed")
        if self._rank == 0:
            print("[Freeze] out_pos_embed frozen (decoder 输入侧)")

        # 7. 冻结 decoder 条件侧: time_emb (SinusoidalPosEmb 无可训练参数, 确保无泄漏)
        _freeze(model.time_emb, "time_emb")
        if self._rank == 0:
            has_params = sum(p.numel() for p in model.time_emb.parameters()) > 0
            print(f"[Freeze] time_emb frozen (decoder 条件侧, has_params={has_params})")

        # 8. 冻结 decoder 输出侧: 顶层独立 layernorm
        _freeze(model.layernorm, "layernorm")
        if self._rank == 0:
            print("[Freeze] layernorm frozen (decoder 输出侧)")

        # 9. 冻结 critic_head
        if getattr(self.grpo_cfg, 'freeze_critic', True):
            _freeze(model.critic_head, "critic_head")
            if self._rank == 0:
                print("[Freeze] critic_head frozen")

        # 10. 冻结 SFT 辅助头 (GRPO 不使用)
        _freeze(model.pixel_aux_head, "pixel_aux_head")
        _freeze(model.image_aux_head, "image_aux_head")
        if self._rank == 0:
            print("[Freeze] SFT aux heads frozen (pixel_aux_head + image_aux_head)")

        # 统计可训练参数
        for p in model.parameters():
            if p.requires_grad:
                trainable_count += p.numel()
        if self._rank == 0:
            print(f"[Freeze] Frozen: {frozen_count:,}, Trainable: {trainable_count:,}")
            # 打印可训练模块
            trainable_names = []
            for name, p in model.named_parameters():
                if p.requires_grad:
                    trainable_names.append(name)
            print(f"[Freeze] Trainable modules: {trainable_names}")

            # 断言验证: 只有 decoder.layers[freeze_dit:] 和 action_head 应可训练
            # 其他任何可训练模块意味着冻结不完整, 会导致意外参数更新
            freeze_up_to = getattr(self.grpo_cfg, 'freeze_dit_layers', 0)
            allowed_prefixes = []
            if freeze_up_to > 0:
                allowed_prefixes.append(f"decoder.layers.{freeze_up_to}")
            allowed_prefixes.append("action_head")
            unexpected = [
                n for n in trainable_names
                if not any(n.startswith(p) or n.startswith(f"module.{p}") for p in allowed_prefixes)
            ]
            if unexpected:
                print(f"[Freeze WARNING] Unexpected trainable modules outside decoder.layers[{freeze_up_to}:] + action_head: {unexpected}")
                print(f"[Freeze WARNING] These modules will be updated during training, which may cause instability!")

    # ------------------------------------------------------------------
    # 数据采集：使用冻结的 π_old 收集 episodes
    # ------------------------------------------------------------------

    def collect_episodes_to_buffer(
        self,
        num_scenes: int,
        num_episodes: int,
        trajectories_per_episode: int = 16,
    ):
        """使用冻结的 π_old 收集 episodes 到缓冲区.

        Args:
            num_scenes: 选择的场景数量
            num_episodes: 要收集的 episode 总数
            trajectories_per_episode: 每个 episode 包含的轨迹数量 (默认16)
        """
        # 清空缓冲区
        self._episode_buffer.clear()

        # DDP 安全: 仅 rank 0 执行采样, 结果广播到所有 rank
        is_ddp = dist.is_initialized()
        rank = dist.get_rank() if is_ddp else 0

        # 选择场景并获取 episode 索引
        rng = np.random.default_rng()  # 所有 rank 共享同一个 rng (过滤逻辑在广播后执行)

        # ---- 是否启用特殊轨迹混入 (掉头 / 大角度拐弯) ----
        special_index_path = getattr(self.grpo_cfg, 'special_trajectory_index_path', None)
        uturn_ratio = float(getattr(self.grpo_cfg, 'uturn_mix_ratio', 0.0) or 0.0)
        sharp_ratio = float(getattr(self.grpo_cfg, 'sharp_turn_mix_ratio', 0.0) or 0.0)
        allow_fallback = bool(getattr(self.grpo_cfg, 'special_allow_cross_scene_fallback', True))
        use_mixed_sampling = (
            special_index_path is not None
            and (uturn_ratio > 0.0 or sharp_ratio > 0.0)
            and hasattr(self.train_dataset, 'sample_mixed_episodes_from_scenes')
        )

        # 首次启用时加载索引文件 (所有 rank 都加载, 这样 trainer 后续日志统计无需 broadcast)
        if use_mixed_sampling and not getattr(self.train_dataset, 'has_special_indices', False):
            try:
                stats = self.train_dataset.load_special_indices(special_index_path)
                if self._rank == 0:
                    print(f"[Special Index] stats={stats}")
            except Exception as e:
                if self._rank == 0:
                    print(f"[Special Index][ERROR] 加载失败, 回退到普通采样: {e!r}")
                use_mixed_sampling = False

        # 默认占位, 后面 broadcast 后会被覆盖
        mix_counts = {"normal": 0, "uturn": 0, "sharp_turn": 0, "fallback": 0}

        # mixed_sampled: List[Tuple[int, Optional[Dict]]]
        #   - (ep_idx, None)                → normal, 走 __getitem__
        #   - (ep_idx, {"ep":..,"s":..,"t":..}) → special, 走 __getitem_segment__
        mixed_sampled = []

        if rank == 0:
            if use_mixed_sampling:
                mixed_sampled, scene_ids, mix_counts = self.train_dataset.sample_mixed_episodes_from_scenes(
                    num_scenes=num_scenes,
                    num_episodes=num_episodes,
                    uturn_ratio=uturn_ratio,
                    sharp_turn_ratio=sharp_ratio,
                    rng=rng,
                    allow_cross_scene_fallback=allow_fallback,
                    return_scene_ids=True,
                )
                print(
                    f"[Data Collection] Mixed sampling: "
                    f"normal={mix_counts['normal']} uturn={mix_counts['uturn']} "
                    f"sharp_turn={mix_counts['sharp_turn']} "
                    f"(target_uturn_ratio={uturn_ratio:.2f}, target_sharp_ratio={sharp_ratio:.2f}, "
                    f"cross_scene_fallback={mix_counts['fallback']})"
                )
            else:
                ep_indices, scene_ids = self.train_dataset.sample_episodes_from_scenes(
                    num_scenes=num_scenes,
                    num_episodes=num_episodes,
                    rng=rng,
                    return_scene_ids=True,
                )
                # 非 mixed: 全部为 (ep_idx, None)
                mixed_sampled = [(int(i), None) for i in ep_indices]

            # 序列化为 DDP 广播准备: indices_tensor + seg_s_tensor + seg_t_tensor
            n = len(mixed_sampled)
            indices_arr = np.array([ep for ep, _ in mixed_sampled], dtype=np.int64)
            # segment 信息: s=-1 / t=-1 表示 None (即 normal, 走 __getitem__)
            seg_s_arr = np.array(
                [seg["s"] if seg is not None else -1 for _, seg in mixed_sampled],
                dtype=np.int64,
            )
            seg_t_arr = np.array(
                [seg["t"] if seg is not None else -1 for _, seg in mixed_sampled],
                dtype=np.int64,
            )
            scene_ids_str = json.dumps(scene_ids)
        else:
            indices_arr = np.array([], dtype=np.int64)
            seg_s_arr = np.array([], dtype=np.int64)
            seg_t_arr = np.array([], dtype=np.int64)
            scene_ids_str = ""

        # 广播数据 (DDP)
        if is_ddp:
            # 广播长度
            length = torch.tensor([len(indices_arr)], dtype=torch.long).cuda()
            dist.broadcast(length, src=0)
            n_samples = length.item()

            # 广播 indices
            if rank != 0:
                indices_arr = np.zeros(n_samples, dtype=np.int64)
            indices_t = torch.from_numpy(indices_arr).cuda()
            dist.broadcast(indices_t, src=0)
            indices_arr = indices_t.cpu().numpy()

            # 广播 seg_s
            if rank != 0:
                seg_s_arr = np.zeros(n_samples, dtype=np.int64)
            seg_s_t = torch.from_numpy(seg_s_arr).cuda()
            dist.broadcast(seg_s_t, src=0)
            seg_s_arr = seg_s_t.cpu().numpy()

            # 广播 seg_t
            if rank != 0:
                seg_t_arr = np.zeros(n_samples, dtype=np.int64)
            seg_t_t = torch.from_numpy(seg_t_arr).cuda()
            dist.broadcast(seg_t_t, src=0)
            seg_t_arr = seg_t_t.cpu().numpy()

            # 广播场景ID
            scene_len = torch.tensor([len(scene_ids_str)], dtype=torch.long).cuda()
            dist.broadcast(scene_len, src=0)
            scene_len_int = scene_len.item()

            if rank == 0:
                scene_bytes = torch.tensor([ord(c) for c in scene_ids_str], dtype=torch.uint8).cuda()
            else:
                scene_bytes = torch.zeros(scene_len_int, dtype=torch.uint8).cuda()
            dist.broadcast(scene_bytes, src=0)

            if rank != 0:
                scene_ids_str = ''.join([chr(c) for c in scene_bytes.cpu().numpy()])

        scene_ids = json.loads(scene_ids_str)

        # 反序列化 mixed_sampled
        mixed_sampled = []
        for i in range(len(indices_arr)):
            ep = int(indices_arr[i])
            s_val = int(seg_s_arr[i])
            t_val = int(seg_t_arr[i])
            if s_val >= 0 and t_val >= 0:
                mixed_sampled.append((ep, {"ep": ep, "s": s_val, "t": t_val}))
            else:
                mixed_sampled.append((ep, None))

        if self._rank == 0:
            print(f"[Data Collection] {num_episodes} episodes x {trajectories_per_episode} trajs from {num_scenes} scenes")

        # 使用 π_old 为每个 episode 采样轨迹
        model = self._old_model
        device = self.model_device
        cfg = self.grpo_cfg

        # 航向角过滤配置
        max_heading_angle_deg = getattr(cfg, 'max_heading_angle_deg', None)
        max_heading_angle_rad = None
        if max_heading_angle_deg is not None and max_heading_angle_deg > 0:
            max_heading_angle_rad = math.radians(max_heading_angle_deg)
            if self._rank == 0:
                print(f"[Data Collection] Heading angle filter enabled: max {max_heading_angle_deg}°")

        # 构建候选池用于重采样 (航向角过滤时需要)
        # 从数据集的所有 episode 中构建候选池，而不仅限于已选中的 indices，
        # 否则多个 episode 被过滤后候选池会迅速耗尽
        dataset_len = len(self.train_dataset)
        # 使用所有索引作为候选池 (排除已选中的，避免重复)
        used_indices = set(ep for ep, _ in mixed_sampled)
        candidate_pool = set(range(dataset_len)) - used_indices
        skipped_count = 0
        
        # 累积奖励统计
        all_rewards_list: List[torch.Tensor] = []
        all_reward_details = {
            "collision": [],
            "occupancy": [],
            "success": [],
            "progress": [],
            "target_speed": [],
            "smooth": [],
            "terminal_heading": [],
            "ttc": [],
            "goal_direction_projection": [],
        }

        # ---- 航向角过滤（所有 rank 一致执行，基于相同的 mixed_sampled） ----
        # 仅 rank 0 执行过滤逻辑，但过滤后的结果必须重新广播到所有 rank
        # 注意: special segment (seg is not None) 不参与航向角过滤, 因为
        # 它的 s/t 是精确指定的, 不应被替换为其他 episode.
        if max_heading_angle_rad is not None:
            if rank == 0:
                for i in range(len(mixed_sampled)):
                    ep, seg = mixed_sampled[i]
                    # special segment 不参与航向角过滤
                    if seg is not None:
                        continue
                    sample = self.train_dataset[ep]
                    point_goal = sample[0]
                    if isinstance(point_goal, torch.Tensor):
                        goal_x = point_goal[0].item()
                        goal_y = point_goal[1].item()
                    else:
                        goal_x = float(point_goal[0])
                        goal_y = float(point_goal[1])
                    heading_angle = abs(math.atan2(goal_y, goal_x))
                    if heading_angle > max_heading_angle_rad:
                        resampled = False
                        pool_list = list(candidate_pool)
                        rng.shuffle(pool_list)
                        for alt_idx in pool_list:
                            alt_sample = self.train_dataset[alt_idx]
                            alt_pg = alt_sample[0]
                            if isinstance(alt_pg, torch.Tensor):
                                alt_x = alt_pg[0].item()
                                alt_y = alt_pg[1].item()
                            else:
                                alt_x = float(alt_pg[0])
                                alt_y = float(alt_pg[1])
                            alt_angle = abs(math.atan2(alt_y, alt_x))
                            if alt_angle <= max_heading_angle_rad:
                                mixed_sampled[i] = (alt_idx, None)
                                candidate_pool.discard(alt_idx)
                                used_indices.add(alt_idx)
                                resampled = True
                                break
                        if not resampled:
                            print(f"[Heading Filter] No valid episode found in candidate pool, keeping original idx={ep}")
                        skipped_count += 1

            # DDP 同步：航向角过滤后的最终 mixed_sampled 必须重新广播
            if is_ddp:
                n_ms = len(mixed_sampled)
                ms_indices = np.array([ep for ep, _ in mixed_sampled], dtype=np.int64)
                ms_seg_s = np.array([seg["s"] if seg is not None else -1 for _, seg in mixed_sampled], dtype=np.int64)
                ms_seg_t = np.array([seg["t"] if seg is not None else -1 for _, seg in mixed_sampled], dtype=np.int64)

                if rank != 0:
                    ms_indices = np.zeros(n_ms, dtype=np.int64)
                    ms_seg_s = np.zeros(n_ms, dtype=np.int64)
                    ms_seg_t = np.zeros(n_ms, dtype=np.int64)

                ms_indices_t = torch.from_numpy(ms_indices).cuda()
                ms_seg_s_t = torch.from_numpy(ms_seg_s).cuda()
                ms_seg_t_t = torch.from_numpy(ms_seg_t).cuda()
                dist.broadcast(ms_indices_t, src=0)
                dist.broadcast(ms_seg_s_t, src=0)
                dist.broadcast(ms_seg_t_t, src=0)

                if rank != 0:
                    ms_indices = ms_indices_t.cpu().numpy()
                    ms_seg_s = ms_seg_s_t.cpu().numpy()
                    ms_seg_t = ms_seg_t_t.cpu().numpy()
                    mixed_sampled = []
                    for j in range(n_ms):
                        ep = int(ms_indices[j])
                        sv = int(ms_seg_s[j])
                        tv = int(ms_seg_t[j])
                        if sv >= 0 and tv >= 0:
                            mixed_sampled.append((ep, {"ep": ep, "s": sv, "t": tv}))
                        else:
                            mixed_sampled.append((ep, None))

        for episode_idx in range(len(mixed_sampled)):
            dataset_idx, seg = mixed_sampled[episode_idx]

            # 从 dataset 获取单个样本的观测数据
            # special segment → __getitem_segment__(ep, s, t)
            # normal          → __getitem__(ep)
            if seg is not None and seg.get("s", -1) >= 0 and seg.get("t", -1) >= 0:
                sample = self.train_dataset.__getitem_segment__(
                    dataset_idx, pixel_start=seg["s"], target=seg["t"]
                )
            else:
                sample = self.train_dataset[dataset_idx]

            # 构建 observation 字典
            # dataset.__getitem__ 返回 tuple:
            #   (point_goal, image_goal, pixel_goal, memory_images, depth_image,
            #    pred_actions, augment_actions, pred_critic, augment_critic, pixel_flag, 
            #    obstacle_points, world_obstacle_points, base_frame_extrinsic, base_extrinsic,
            #    inflation_local_points, world_inflation_points,
            #    camera_intrinsic, camera_extrinsic)
            observation = {
                "rgb": sample[3],       # memory_images
                "depth": sample[4],     # depth_image
                "goal_point": sample[0],  # point_goal
                "goal_image": sample[1],  # image_goal
                "goal_pixel": sample[2],  # pixel_goal
                # 相机参数（用于投影障碍物奖励）
                "camera_intrinsic": sample[16] if len(sample) > 16 else None,  # [3, 3]
                "camera_extrinsic": sample[17] if len(sample) > 17 else None,  # [4, 4] 深度图同帧的相机外参
                "base_frame_extrinsic": sample[12] if len(sample) > 12 else None,  # [4, 4]
                "base_extrinsic": sample[13] if len(sample) > 13 else None,      # [4, 4]
            }

            # 将观测数据扩展为 batch (trajectories_per_episode 条轨迹)
            # 每条轨迹需要独立的观测，但它们共享同一个观测
            # 使用 repeat 而非 expand: expand 是只读视图，repeat 分配新内存更安全
            def _expand_obs(t, size):
                if t is None:
                    return None
                if isinstance(t, torch.Tensor):
                    return t.unsqueeze(0).repeat(size, *([1] * t.dim()))
                return t

            batch_obs = {
                "rgb": _expand_obs(observation["rgb"], trajectories_per_episode),
                "depth": _expand_obs(observation["depth"], trajectories_per_episode),
                "goal_point": _expand_obs(observation["goal_point"], trajectories_per_episode),
                "goal_image": _expand_obs(observation["goal_image"], trajectories_per_episode),
                "goal_pixel": _expand_obs(observation["goal_pixel"], trajectories_per_episode),
            }

            # 移动到 device
            batch_obs_dev = {
                k: (v.to(device, non_blocking=True) if torch.is_tensor(v) else v)
                for k, v in batch_obs.items()
            }

            # 使用 π_old 采样 trajectories_per_episode 条轨迹
            with torch.no_grad():
                rollout_out = model.sample_actions_with_logprob(
                    input_images=batch_obs_dev["rgb"],
                    input_depths=batch_obs_dev["depth"],
                    goal_point=batch_obs_dev.get("goal_point"),
                    goal_image=batch_obs_dev.get("goal_image"),
                    goal_pixel=batch_obs_dev.get("goal_pixel"),
                    goal_type=cfg.goal_type,
                    sample_num=1,
                    num_inference_steps=cfg.rollout_num_inference_steps,
                    temperature=cfg.rollout_temperature,
                )

            # 提取轨迹数据
            actions = rollout_out["actions"]  # [trajectories_per_episode, predict_size, 3]
            chains = rollout_out["chains"]    # [T+1, trajectories_per_episode, pred, 3]
            timesteps = rollout_out["timesteps"]  # [T]

            # 计算 old_logprobs (使用 π_old 在采样链上计算)
            # 需要重新计算 logprob，因为 sample_actions_with_logprob 返回的是采样时的 logprob
            # 对于扩散模型，我们需要在完整链上计算
            goal_embed = rollout_out["goal_embed"]  # [trajectories_per_episode, 1, token_dim]
            rgbd_embed = rollout_out["rgbd_embed"]  # [trajectories_per_episode, seq, token_dim]

            # 重算 old_logprobs (每个去噪步的 logprob)
            T = timesteps.shape[0]
            if cfg.sample_step_subset is not None and cfg.sample_step_subset < T:
                step_indices = torch.arange(T - cfg.sample_step_subset, T, device=device)
            else:
                step_indices = torch.arange(T, device=device)

            with torch.no_grad():
                old_lp_out = model.compute_logprob_value(
                    chains=chains,
                    timesteps=timesteps,
                    goal_embed=goal_embed,
                    rgbd_embed=rgbd_embed,
                    step_indices=step_indices,
                    joint_logprob=True,
                    temperature=cfg.rollout_temperature,
                )
                old_logprobs_raw = old_lp_out["logprobs"]  # [T_used, trajectories_per_episode]

                # 按轨迹保存每个去噪步的 old logprob，训练时逐步做 ratio/clip。
                old_logprobs = old_logprobs_raw.transpose(0, 1)  # [trajectories_per_episode, T_used]

            # 构建轨迹列表用于奖励计算
            # dataset.__getitem__ 返回 tuple:
            #   (point_goal, image_goal, pixel_goal, memory_images, depth_image,
            #    pred_actions, augment_actions, pred_critic, augment_critic, pixel_flag, 
            #    obstacle_local_points, world_obstacle_points, base_frame_extrinsic, base_extrinsic,
            #    inflation_local_points, world_inflation_points,
            #    camera_intrinsic, camera_extrinsic)
            trajectories = []
            for i in range(trajectories_per_episode):
                traj = {
                    "actions": actions[i],
                    "goal_point": observation["goal_point"],  # 保留完整的 [x, y, theta] 3维目标点
                    "goal_image": observation["goal_image"],
                    "goal_pixel": observation["goal_pixel"],
                    "gt_actions": sample[5],  # pred_actions (GT 标签)
                }
                # 只使用局部坐标膨胀障碍物，不再支持世界坐标转换
                # sample[14] = inflation_local_points [K, 2] 局部坐标系膨胀障碍物（蓝色）
                if len(sample) > 14 and sample[14] is not None:
                    traj["inflation_points"] = sample[14]
                else:
                    raise ValueError(
                        "GRPO训练: 缺少 inflation_points！\n"
                        "障碍物点云必须由数据集预处理，请检查数据集是否正确加载了点云文件。"
                    )
                
                # 添加投影障碍物奖励所需的字段
                # 深度图投影需要: depth_image, camera_intrinsic, camera_extrinsic, base_frame_extrinsic, base_extrinsic
                traj["depth_image"] = observation["depth"]
                traj["camera_intrinsic"] = observation.get("camera_intrinsic")
                traj["camera_extrinsic"] = observation.get("camera_extrinsic")
                traj["base_frame_extrinsic"] = observation.get("base_frame_extrinsic")
                traj["base_extrinsic"] = observation.get("base_extrinsic")
                
                trajectories.append(traj)

            # 计算奖励
            reward_out = self.reward_fn.compute(trajectories, device=device, dtype=torch.float32)
            if isinstance(reward_out, dict):
                rewards = reward_out["reward"].to(device)
                # 提取分项奖励（原始值，未加权，用于日志统计）
                reward_details = {
                    "collision": reward_out.get("collision", torch.zeros_like(rewards)),
                    "occupancy": reward_out.get("occupancy", torch.zeros_like(rewards)),
                    "success": reward_out.get("success", torch.zeros_like(rewards)),
                    "progress": reward_out.get("progress", torch.zeros_like(rewards)),
                    "target_speed": reward_out.get("target_speed", torch.zeros_like(rewards)),
                    "smooth": reward_out.get("smooth", torch.zeros_like(rewards)),
                    "terminal_heading": reward_out.get("terminal_heading", torch.zeros_like(rewards)),
                    "ttc": reward_out.get("ttc", torch.zeros_like(rewards)),
                }
                # 添加投影障碍物奖励（如果有）
                if "projected_collision" in reward_out:
                    reward_details["projected_collision"] = reward_out["projected_collision"]
                if "projected_occupancy" in reward_out:
                    reward_details["projected_occupancy"] = reward_out["projected_occupancy"]
            else:
                rewards = reward_out.to(device)
                reward_details = None

            # ---- 组内优势计算 ----
            # 使用标准的 compute_grpo_advantages 函数
            # 在同观测的 trajectories_per_episode 条轨迹内计算 advantage
            # group_size = trajectories_per_episode (每个观测采样的轨迹数)
            adv_out = compute_grpo_advantages(
                rewards=rewards,
                group_size=trajectories_per_episode,
                normalize=cfg.normalize_advantages,
                eps=1e-8,
            )
            advantages = adv_out["advantages"]

            # 将 episode 数据添加到缓冲区（传入 advantages 而非 rewards）
            self._episode_buffer.add_episode(
                observation=observation,
                trajectories=trajectories,
                advantages=advantages,
                old_logprobs=old_logprobs,
                chains=chains,           # 保存完整扩散链
                timesteps=timesteps,     # 保存时间步
                goal_embed=goal_embed,   # 保存目标嵌入
                rgbd_embed=rgbd_embed,   # 保存 RGB-D 嵌入
                scene_ids=scene_ids,
                reward_details=reward_details,  # 保存分项奖励
            )

            # 打印奖励平均值到训练日志（仅最后一个 episode 打印，避免刷屏）
            if self._rank == 0 and (episode_idx + 1) == len(mixed_sampled):
                avg_reward = rewards.mean().item()
                reward_parts = []
                if reward_details is not None:
                    for k, v in reward_details.items():
                        if v is not None and v.numel() > 0:
                            reward_parts.append(f"{k}={v.mean().item():.4f}")
                detail_str = ", ".join(reward_parts) if reward_parts else ""
                print(f"[Data Collection] Collected {episode_idx + 1}/{len(mixed_sampled)} episodes, avg_reward={avg_reward:.4f} ({detail_str})")
            
            # 累积奖励数据用于总体统计
            all_rewards_list.append(rewards.detach().cpu())
            if reward_details is not None:
                for k, v in reward_details.items():
                    if v is not None and v.numel() > 0:
                        all_reward_details[k].append(v.detach().cpu())

        if self._rank == 0:
            print(f"[Data Collection] Buffer size: {self._episode_buffer.size} trajectories")
            if max_heading_angle_rad is not None and skipped_count > 0:
                print(f"[Data Collection] Heading angle filter: skipped {skipped_count} episodes, resampled from candidate pool")
            
            # 打印总体奖励统计
            if all_rewards_list:
                all_rewards = torch.cat(all_rewards_list, dim=0)
                avg_reward_total = all_rewards.mean().item()
                std_reward_total = all_rewards.std().item()
                
                reward_detail_parts = []
                for k in ["collision", "occupancy", "success", "progress",
                          "target_speed", "smooth", "terminal_heading", "ttc",
                          "goal_direction_projection"]:
                    if all_reward_details[k]:
                        all_k = torch.cat(all_reward_details[k], dim=0)
                        reward_detail_parts.append(f"{k}={all_k.mean().item():.4f}")
                        # 保存分项奖励用于 TensorBoard 记录
                        self._collected_reward_details[k] = all_k.mean().item()
                # 添加投影障碍物奖励的统计
                for k in ["projected_collision", "projected_occupancy"]:
                    if all_reward_details.get(k):
                        all_k = torch.cat(all_reward_details[k], dim=0)
                        reward_detail_parts.append(f"{k}={all_k.mean().item():.4f}")
                        self._collected_reward_details[k] = all_k.mean().item()
                
                # 保存 overall reward 用于 TensorBoard 记录
                self._collected_overall_reward = avg_reward_total
                
                detail_str = ", ".join(reward_detail_parts) if reward_detail_parts else ""
                print(f"[Data Collection] Overall reward statistics: avg={avg_reward_total:.4f}, std={std_reward_total:.4f} ({detail_str})")

        # 数据采集阶段写入 TensorBoard
        if self._tb_writer is not None:
            self._tb_writer.add_scalar("buffer/size", self._episode_buffer.size, self._global_iter)
            # 写入奖励统计到 TensorBoard
            if all_rewards_list:
                all_rewards = torch.cat(all_rewards_list, dim=0)
                self._tb_writer.add_scalar("reward/avg", all_rewards.mean().item(), self._global_iter)
                self._tb_writer.add_scalar("reward/std", all_rewards.std().item(), self._global_iter)
                for k in ["collision", "occupancy", "success", "progress",
                          "target_speed", "smooth", "terminal_heading", "ttc",
                          "goal_direction_projection"]:
                    if all_reward_details[k]:
                        all_k = torch.cat(all_reward_details[k], dim=0)
                        self._tb_writer.add_scalar(f"reward/{k}", all_k.mean().item(), self._global_iter)
                # 添加投影障碍物奖励的 TensorBoard 记录
                for k in ["projected_collision", "projected_occupancy"]:
                    if all_reward_details.get(k):
                        all_k = torch.cat(all_reward_details[k], dim=0)
                        self._tb_writer.add_scalar(f"reward/{k}", all_k.mean().item(), self._global_iter)
            self._tb_writer.flush()

    # ------------------------------------------------------------------
    # 核心 GRPO 迭代
    # ------------------------------------------------------------------

    def grpo_iteration_new(
        self,
        batch: Dict[str, torch.Tensor],
        do_optimizer_step: bool = True,
    ) -> Dict[str, torch.Tensor]:
        """新版 GRPO 迭代，适配新的数据流.

        新流程：
        1. 从缓冲区采样的轨迹已包含观测、动作、组内优势和每步 old_logprob
        2. 直接使用已计算的组内优势进行策略更新，不再做跨组归一化
        3. 损失计算：ratio = π_θ / π_old，执行 GRPO 裁剪

        Args:
            batch: 从缓冲区采样的轨迹数据，每项 shape [batch_size, ...]
                   包含: rgb, depth, actions, advantage, old_logprob 等
            do_optimizer_step: 是否执行 optimizer.step()

        Returns:
            损失字典 (用于日志)
        """
        model = self.model
        if hasattr(model, 'module'):
            model = model.module  # unwrap DDP
        # PPO/GRPO ratio 应只反映策略参数变化；关闭 dropout 等训练态随机性，
        # 但不使用 no_grad，因此 logprob 仍正常反传到可训练参数。
        model.eval()
        cfg = self.grpo_cfg
        device = self.model_device

        # 从 batch 中提取数据
        # batch 包含: rgb, depth, actions, advantage, old_logprob 等
        B = batch["rgb"].shape[0]  # batch_size (64条轨迹)

        # 移动到 device
        rgb = batch["rgb"].to(device, non_blocking=True)
        depth = batch["depth"].to(device, non_blocking=True)
        actions = batch["actions"].to(device, non_blocking=True)  # [B, predict_size, 3]
        advantages = batch["advantage"].to(device, non_blocking=True)  # [B] — 已在组内计算完毕
        old_logprobs = batch["old_logprob"].to(device, non_blocking=True)  # [B, T_used]

        goal_point = batch.get("goal_point")
        if goal_point is not None:
            goal_point = goal_point.to(device, non_blocking=True)
        goal_image = batch.get("goal_image")
        if goal_image is not None:
            goal_image = goal_image.to(device, non_blocking=True)
        goal_pixel = batch.get("goal_pixel")
        if goal_pixel is not None:
            goal_pixel = goal_pixel.to(device, non_blocking=True)

        # ---- 优势已在上游（同观测组内）计算完毕，直接使用 ----
        # 不再对抽取的64条轨迹做跨组归一化

        # ---- 重新计算当前策略的 logprob ----
        # 使用存储的扩散链计算当前策略的 logprob
        chains = batch["chain"].to(device, non_blocking=True)  # [B, T+1, pred, 3]
        timesteps = batch["timesteps"][0].to(device, non_blocking=True)  # [T]，所有轨迹共享
        goal_embed = batch["goal_embed"].to(device, non_blocking=True)  # [B, 1, token_dim]
        rgbd_embed = batch["rgbd_embed"].to(device, non_blocking=True)  # [B, seq, token_dim]

        # 转置 chains 以匹配 compute_logprob_value 的输入格式
        # compute_logprob_value 期望: [T+1, B, pred, 3]
        chains_transposed = chains.transpose(0, 1)  # [T+1, B, pred, 3]

        # 计算当前策略的 logprob
        T = timesteps.shape[0]
        if cfg.sample_step_subset is not None and cfg.sample_step_subset < T:
            step_indices = torch.arange(T - cfg.sample_step_subset, T, device=device)
        else:
            step_indices = torch.arange(T, device=device)

        logprob_out = model.compute_logprob_value(
            chains=chains_transposed,
            timesteps=timesteps,
            goal_embed=goal_embed,
            rgbd_embed=rgbd_embed,
            step_indices=step_indices,
            joint_logprob=True,
            temperature=cfg.rollout_temperature,
        )
        logprobs_raw = logprob_out["logprobs"]  # [T_used, B]
        if old_logprobs.shape != (B, logprobs_raw.shape[0]):
            raise ValueError(
                f"old_logprobs shape {tuple(old_logprobs.shape)} does not match "
                f"current logprobs shape {tuple(logprobs_raw.shape)}"
            )

        # ---- KL-loss 计算 ----
        # 使用 _ref_model 计算参考策略的 logprob。
        # KL 同样使用整条去噪链的联合概率（两种 clip_ratio_mode 共享此 KL 计算）。
        kl_loss = torch.tensor(0.0, device=device)
        if self._ref_model is not None:
            with torch.no_grad():
                ref_lp_out = self._ref_model.compute_logprob_value(
                    chains=chains_transposed,
                    timesteps=timesteps,
                    goal_embed=goal_embed,
                    rgbd_embed=rgbd_embed,
                    step_indices=step_indices,
                    joint_logprob=True,
                    temperature=cfg.rollout_temperature,
                )
                ref_logprobs_raw = ref_lp_out["logprobs"]  # [T_used, B]
                ref_logprobs = ref_logprobs_raw.sum(dim=0)  # [B]
            # KL(π_θ || π_ref) 估计: exp(ref - logp) - (ref - logp) - 1
            logprobs_for_kl = logprobs_raw.sum(dim=0)  # [B] — 整条链联合 logprob
            kl_delta = ref_logprobs.detach() - logprobs_for_kl
            kl_elem = torch.exp(kl_delta) - kl_delta - 1.0
            kl_loss = kl_elem.mean()

        # ---- clip ratio 模式选择 ----
        clip_ratio_mode = getattr(cfg, 'clip_ratio_mode', 'joint')

        if clip_ratio_mode == "joint":
            # ---- 模式1: 整条去噪链联合 logprob (当前现状) ----
            # 对每个去噪步的 logprob 求和, 得到整条链上动作序列的联合 logprob,
            # 再基于此计算 ratio 与 PPO-clip。
            logprobs = logprobs_raw.sum(dim=0)  # [B]
            old_logprobs_sum = old_logprobs.sum(dim=1)  # [B]

            ratio = torch.exp(logprobs - old_logprobs_sum)  # [B]
            clipped_ratio = torch.clamp(ratio, 1.0 - cfg.clip_ratio_low, 1.0 + cfg.clip_ratio_high)
            pg_loss1 = -advantages * ratio  # [B]
            pg_loss2 = -advantages * clipped_ratio  # [B]
            pg_loss = torch.max(pg_loss1, pg_loss2).mean()

        elif clip_ratio_mode == "stepwise":
            # ---- 模式2: 每个去噪步单独计算 ratio + clip, 最后取平均 ----
            # logprobs_raw: [T_used, B], old_logprobs: [B, T_used]
            # 对每个去噪步 t:
            #   ratio_t = exp(logprob_t - old_logprob_t)
            #   clipped_ratio_t = clamp(ratio_t, 1-eps_low, 1+eps_high)
            #   pg_loss_t = max(-adv * ratio_t, -adv * clipped_ratio_t)
            # 最终 pg_loss = mean over T_used and B
            stepwise_ratio = torch.exp(logprobs_raw - old_logprobs.T)  # [T_used, B]
            stepwise_clipped = torch.clamp(
                stepwise_ratio, 1.0 - cfg.clip_ratio_low, 1.0 + cfg.clip_ratio_high
            )  # [T_used, B]
            # advantages: [B] -> 广播到 [T_used, B]
            pg_loss1_per_step = -advantages.unsqueeze(0) * stepwise_ratio  # [T_used, B]
            pg_loss2_per_step = -advantages.unsqueeze(0) * stepwise_clipped  # [T_used, B]
            pg_loss_per_step = torch.max(pg_loss1_per_step, pg_loss2_per_step)  # [T_used, B]
            pg_loss = pg_loss_per_step.mean()  # 对所有步和所有轨迹取平均

            # 为了日志一致性, 计算 ratio 的平均值
            ratio = stepwise_ratio.mean(dim=0)  # [B] — 每条轨迹各步 ratio 的均值 (仅供日志)
        else:
            raise ValueError(f"不支持的 clip_ratio_mode: {clip_ratio_mode}, 请选择 'joint' 或 'stepwise'")

        loss = pg_loss + cfg.kl_beta * kl_loss

        # 反向传播
        loss_scaled = loss / cfg.gradient_accumulation_steps
        loss_scaled.backward()

        # 梯度累积
        if do_optimizer_step:
            if cfg.max_grad_norm > 0:
                torch.nn.utils.clip_grad_norm_(filter(lambda p: p.requires_grad, model.parameters()), cfg.max_grad_norm)
            self.optimizer.step()
            self.optimizer.zero_grad(set_to_none=True)

        # ---- 构建详细统计指标 ----
        # 基础指标
        metrics = {
            "loss": loss.item(),
            "pg_loss": pg_loss.item(),
            "kl_loss": kl_loss.item(),
            "ratio": ratio.mean().item(),
        }

        # 分项奖励统计（原始值，未加权）
        reward_keys = ["collision", "occupancy", "success", "progress",
                       "target_speed", "smooth", "terminal_heading",
                       "projected_collision", "projected_occupancy", "ttc"]
        for k in reward_keys:
            key = f"reward_{k}"
            if key in batch:
                vals = batch[key]
                # 过滤掉 None 值
                valid_vals = [v for v in vals if v is not None]
                if len(valid_vals) > 0:
                    vals_tensor = torch.stack(valid_vals) if isinstance(valid_vals[0], torch.Tensor) else torch.tensor(valid_vals)
                    metrics[f"{k}_mean"] = vals_tensor.mean().item()
                    metrics[f"{k}_std"] = vals_tensor.std().item()
                else:
                    metrics[f"{k}_mean"] = 0.0
                    metrics[f"{k}_std"] = 0.0

        self._last_loss_metrics = {
            "loss/total": loss.item(),
            "loss/pg_loss": pg_loss.item(),
            "loss/kl_loss": kl_loss.item(),
            # grpo 前缀的指标（与 loss/ 前缀分开，便于 TensorBoard 分类）
            "grpo/pg_loss": pg_loss.item(),
            "grpo/kl_loss": kl_loss.item(),
            "grpo/ratio": ratio.mean().item(),
            **{f"grpo/{k}": v for k, v in metrics.items() if k not in ['loss', 'pg_loss', 'kl_loss', 'ratio']},
            # 分项奖励
            "grpo/collision_mean": metrics.get("collision_mean", 0.0),
            "grpo/occupancy_mean": metrics.get("occupancy_mean", 0.0),
            "grpo/success_mean": metrics.get("success_mean", 0.0),
            "grpo/progress_mean": metrics.get("progress_mean", 0.0),
            "grpo/target_speed_mean": metrics.get("target_speed_mean", 0.0),
            "grpo/smooth_mean": metrics.get("smooth_mean", 0.0),
            "grpo/terminal_heading_mean": metrics.get("terminal_heading_mean", 0.0),
            "grpo/projected_collision_mean": metrics.get("projected_collision_mean", 0.0),
            "grpo/projected_occupancy_mean": metrics.get("projected_occupancy_mean", 0.0),
            "grpo/ttc_mean": metrics.get("ttc_mean", 0.0),
        }

        return self._last_loss_metrics

    # ------------------------------------------------------------------
    # 训练循环覆写 (不依赖 HuggingFace Trainer 的 epoch/step 语义)
    # ------------------------------------------------------------------

    def train(self, resume_from_checkpoint=None, **kwargs):
        """新版 GRPO 主循环.

        训练流程:
            1. 使用冻结的 π_old 在 N 个场景中收集 episode
            2. 将采集的 episode 存入回放缓冲区
            3. 每次从回放缓冲区随机抽取轨迹进行训练
            4. 计算优势：在同观测的轨迹组内归一化，存入 buffer
            5. 损失计算：直接使用组内优势，ratio = π_θ / π_old, GRPO 裁剪
            6. 梯度更新：反向传播更新 π_θ
            7. 每个 generation 完成后: π_old = π_θ (直接替换, 不用最优checkpoint)
            8. best model 维护标准: advantage 平均值
            9. 同一批采样数据训练过程中 π_old 绝对不能变
        """
        device = self.model_device
        cfg = self.grpo_cfg

        # 新版配置参数
        num_scenes = getattr(cfg, 'num_scenes_per_buffer', 4)  # 每轮 4 个场景
        num_episodes_per_round = getattr(cfg, 'num_episodes_per_round', 128)  # 每轮 128 个 episode
        trajectories_per_episode = getattr(cfg, 'trajectories_per_episode', 16)  # 每个 episode 16 条轨迹
        train_batch_size = getattr(cfg, 'train_batch_size', 64)  # 每次训练 64 条轨迹
        # 兼容新旧配置: num_train_epochs_per_round (新版) 或 buffer_epochs (旧版)
        num_train_epochs_per_round = getattr(cfg, 'num_train_epochs_per_round', 
                                             getattr(cfg, 'buffer_epochs', 2))  # 每轮训练epoch数
        checkpoint_every = getattr(cfg, 'checkpoint_every', 5)  # 每 5 代保存 checkpoint

        log_every = cfg.log_every
        grad_accum = cfg.gradient_accumulation_steps

        # 计算每轮的迭代次数: buffer_size / train_batch_size 为1个epoch的iteration数
        num_iters_per_epoch = num_episodes_per_round * trajectories_per_episode // train_batch_size
        num_train_iters_per_round = num_train_epochs_per_round * num_iters_per_epoch

        # 若从 checkpoint 恢复
        start_iter = 0
        if resume_from_checkpoint:
            ckpt_path = self._get_checkpoint_path(resume_from_checkpoint)
            if ckpt_path and os.path.exists(ckpt_path):
                start_iter = self._load_state(ckpt_path)

        self.model.train()
        self._epoch = 0
        self.optimizer.zero_grad(set_to_none=True)

        if self._rank == 0:
            print(
                f"[GRPO] scenes={num_scenes}, episodes={num_episodes_per_round}, "
                f"trajs/ep={trajectories_per_episode}, batch={train_batch_size}, "
                f"epochs/round={num_train_epochs_per_round}, iters/round={num_train_iters_per_round}, "
                f"ckpt_every={checkpoint_every}"
            )

        it = start_iter
        generation = 0  # 代数计数器

        while True:
            # ---- 数据采集：使用冻结的 π_old 收集 episodes ----
            if self._rank == 0:
                print(f"\n{'='*60}")
                print(f"[Generation {generation}] Collecting data...")
                print(f"{'='*60}")

            self.collect_episodes_to_buffer(
                num_scenes=num_scenes,
                num_episodes=num_episodes_per_round,
                trajectories_per_episode=trajectories_per_episode,
            )

            if self._rank == 0:
                print(f"[Buffer] Total trajectories in buffer: {self._episode_buffer.size}")

            # ---- 训练阶段：从缓冲区随机抽取轨迹进行训练 ----
            for train_iter in range(num_train_iters_per_round):
                # 从缓冲区随机抽取 batch_size 条轨迹
                batch = self._episode_buffer.sample_trajectories(train_batch_size)

                if not batch:
                    if self._rank == 0:
                        print(f"[Training] Buffer empty, skipping iteration")
                    break

                # 梯度累积
                is_accum_step = (train_iter + 1) % grad_accum == 0

                # 使用新的训练迭代方法
                metrics = self.grpo_iteration_new(batch, do_optimizer_step=is_accum_step)

                # 日志写入 TensorBoard (每个 step 都写入, 不受 log_every 控制)
                if self._tb_writer is not None:
                    step = self._global_iter
                    for k, v in metrics.items():
                        if not k.startswith("_"):  # 跳过内部指标
                            self._tb_writer.add_scalar(k, v, step)
                    lr = self.optimizer.param_groups[0]['lr']
                    self._tb_writer.add_scalar("lr", lr, step)
                    # 每隔 log_every 步 flush 一次 (避免每步 flush 导致 I/O 瓶颈)
                    if (train_iter + 1) % log_every == 0:
                        self._tb_writer.flush()

                # Print 日志 (受 log_every 控制)
                if self._rank == 0 and (train_iter + 1) % log_every == 0:
                    self._print_log_new(
                        generation=generation,
                        train_iter=train_iter,
                        num_iters=num_train_iters_per_round,
                        metrics=metrics,
                    )

                # LR 步进
                if is_accum_step:
                    self.lr_scheduler.step()

                self._global_iter += 1
                it += 1

            # ---- 本轮训练完成 ----
            if self._rank == 0:
                print(f"\n[Generation {generation} Complete]")

            # ---- 保存 checkpoint (每 checkpoint_every 代) ----
            if generation > 0 and generation % checkpoint_every == 0 and self._rank == 0:
                ckpt_path = os.path.join(
                    self.config.checkpoint_folder,
                    f"grpo_gen_{generation}"
                )
                self.save_model(ckpt_path)
                self._save_state(ckpt_path)
                print(f"[Checkpoint] Generation {generation} saved to {ckpt_path}")

            # ---- 用当前策略 π_θ 直接替换 π_old ----
            # 重要: 同一批采样数据训练过程中 π_old 绝对不能变
            # 每个 generation 训练完成后, π_old = π_θ (当前模型)
            if self._old_model is not None:
                self._update_old_model(None)  # None = 使用当前模型直接替换
                if self._rank == 0:
                    print(f"[π_old] 已替换为当前 π_θ, 下一轮将作为固定旧策略")

            # 写入 TensorBoard
            if self._tb_writer is not None:
                if self._collected_overall_reward is not None:
                    self._tb_writer.add_scalar("generation/overall_reward", self._collected_overall_reward, generation)
                    for k in ["collision", "occupancy", "success", "progress",
                               "target_speed", "smooth", "terminal_heading",
                               "projected_collision", "projected_occupancy", "ttc"]:
                        v = self._collected_reward_details.get(k)
                        if v is not None:
                            self._tb_writer.add_scalar(f"generation/{k}", v, generation)
                self._tb_writer.add_scalar("round/generation", generation, self._global_iter)
                self._tb_writer.flush()
            
            # 清空缓冲区（下一轮重新采集数据）
            self._episode_buffer.clear()

            # 增加代数计数器
            generation += 1

            # 检查是否达到最大代数（可选配置）
            max_generations = getattr(cfg, 'max_generations', None)
            if max_generations is not None and generation >= max_generations:
                if self._rank == 0:
                    print(f"\n[Training Complete] Reached max generations: {max_generations}")
                break

        if self._rank == 0:
            if self._tb_writer is not None:
                self._tb_writer.close()
            print("GRPO training complete.")

    def _print_log_new(
        self,
        generation: int,
        train_iter: int,
        num_iters: int,
        metrics: Dict[str, float],
    ):
        """新版日志打印方法 - 显示详细统计信息."""
        elapsed = time.time() - self._start_time
        lr = self.optimizer.param_groups[0]['lr']

        # ---- 损失信息 ----
        loss_str = f"loss={metrics.get('loss/total', 0):.6f}, pg={metrics.get('loss/pg_loss', 0):.6f}, kl={metrics.get('loss/kl_loss', 0):.6f}"

        # ---- 分项奖励统计 ----
        detail_keys = ["collision", "occupancy", "success", "progress", "ttc"]
        detail_parts = []
        for dk in detail_keys:
            mk = f"grpo/{dk}_mean"
            if mk in metrics:
                detail_parts.append(f"{dk}={metrics[mk]:.3f}")
        detail_str = f"reward({', '.join(detail_parts)})" if detail_parts else ""

        print(
            f"[Gen {generation}, iter {train_iter+1}/{num_iters}] "
            f"lr={lr:.2e}, {loss_str}, "
            f"ratio={metrics.get('grpo/ratio', 1.0):.4f}, {detail_str}, "
            f"time={elapsed:.1f}s"
        )

    # ------------------------------------------------------------------
    # Optimizer / Scheduler
    # ------------------------------------------------------------------

    def create_optimizer(self):
        rank = self._rank
        lr = self.grpo_cfg.lr if self.grpo_cfg.lr is not None else self.il_cfg.lr
        self._max_lr = lr  # 保存初始学习率, 供 create_scheduler 使用
        if rank == 0:
            print(f"[Rank 0] GRPO optimizer lr={lr}")

        model = self.model
        if hasattr(model, 'module'):
            model = model.module

        optimizer = torch.optim.AdamW(
            filter(lambda p: p.requires_grad, model.parameters()),
            lr=lr,
            weight_decay=self.grpo_cfg.weight_decay,
        )
        if rank == 0:
            total_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
            print(f"[Rank 0] Total trainable params: {total_params:,}")
        return optimizer

    def create_scheduler(self, optimizer, num_training_steps: int):
        max_lr = self._max_lr
        min_lr = self.grpo_cfg.min_lr if self.grpo_cfg.min_lr is not None else max_lr * 0.5
        if self._rank == 0:
            print(f"[Rank 0] LR scheduler: LinearLR from {max_lr:.2e} -> {min_lr:.2e} over {num_training_steps} steps")
        return torch.optim.lr_scheduler.LinearLR(
            optimizer,
            start_factor=1.0,
            end_factor=min_lr / max_lr if max_lr > 0 else 1.0,
            total_iters=num_training_steps,
        )

    def create_optimizer_and_scheduler(self, num_training_steps: int):
        print("\n=== create GRPO optimizer and scheduler ===")
        self.optimizer = self.create_optimizer()
        self.lr_scheduler = self.create_scheduler(self.optimizer, num_training_steps)
        return self.optimizer, self.lr_scheduler

    # ------------------------------------------------------------------
    # 保存 / 加载
    # ------------------------------------------------------------------

    def save_model(self, output_dir: str, **kwargs):
        if hasattr(self.model, 'module'):
            model_to_save = self.model.module
        else:
            model_to_save = self.model
        os.makedirs(output_dir, exist_ok=True)
        torch.save(model_to_save.state_dict(), os.path.join(output_dir, "navdp.ckpt"))
        print(f"[Rank {self._rank}] Saved model to {output_dir}")

    def _get_checkpoint_path(self, resume_from_checkpoint):
        if isinstance(resume_from_checkpoint, str):
            if os.path.isdir(resume_from_checkpoint):
                return resume_from_checkpoint
            if os.path.isdir(os.path.join(self.config.checkpoint_folder, resume_from_checkpoint)):
                return os.path.join(self.config.checkpoint_folder, resume_from_checkpoint)
        return None

    def _load_state(self, ckpt_path: str) -> int:
        state_file = os.path.join(ckpt_path, "grpo_state.pt")
        if not os.path.exists(state_file):
            return 0
        state = torch.load(state_file, map_location=self.model_device)
        self._global_iter = state.get("global_iter", 0)
        self.optimizer.load_state_dict(state["optimizer"])
        if "lr_scheduler" in state:
            self.lr_scheduler.load_state_dict(state["lr_scheduler"])
        print(f"[Rank {self._rank}] Resumed from iter {self._global_iter}")
        return self._global_iter

    def _save_state(self, output_dir: str):
        state = {
            "global_iter": self._global_iter,
            "optimizer": self.optimizer.state_dict(),
            "lr_scheduler": self.lr_scheduler.state_dict(),
        }
        torch.save(state, os.path.join(output_dir, "grpo_state.pt"))
