"""NavDP + GRPO 模型层.

参考 RLinf FlowSDE 的 sample_actions / get_log_prob_value 设计, 但 NavDP 使用
DDPM 离散调度器 (epsilon 预测), 因此采用 DDPO/DPOK 范式: 利用 DDPM 反向过程
``p(x_{t-1} | x_t) = N(mu_theta, sigma_t^2 * I)`` 自带的高斯 log-prob 作为
策略的对数概率, 不改写为连续 Flow-SDE.

关键约定:
- ``sample_actions_with_logprob`` 返回 chain 长度 = ``num_inference_steps``,
  其中 ``chains[k]`` 表示反向时间步 ``k`` 时的样本 (k=0 是初始噪声, k=T 是最终动作).
- ``prev_logprobs[k]`` 表示从 ``chains[k]`` -> ``chains[k+1]`` 这一步的高斯 log-prob.
- 训练侧 ``compute_logprob_value`` 用同一份 ``chains`` 与 ``timesteps`` 重新前向,
  并在 *相同的 x_t* 上计算新策略下的 ``logprobs``, 形状与 ``prev_logprobs`` 完全一致.
"""

import math
import os
from typing import Any, Dict, List, Optional

# ================================================================
# 彻底禁止 HuggingFace 联网请求
# ================================================================
os.environ.setdefault("HF_DATASETS_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("HF_HYDRA_FULL_ERROR", "1")

import torch

from autonomy_internnav.train.navdp_model import NavDPModelConfig, NavDPNet


# ---------------------------------------------------------------------------
# DDPM 反向高斯参数工具函数
# ---------------------------------------------------------------------------


def _ddpm_posterior_params(
    scheduler,
    model_output: torch.Tensor,
    timestep: int,
    sample: torch.Tensor,
    eps: float = 1e-12,
) -> Dict[str, torch.Tensor]:
    """根据 diffusers DDPMScheduler 的 step() 公式重建 ``q(x_{t-1}|x_t,x_0)`` 的
    高斯均值与标准差.

    与 ``scheduler.step(model_output, t, sample).prev_sample`` 数学等价 (确定性
    模式下返回 mean; 随机模式下 prev_sample = mean + std * eps).
    采用 epsilon-prediction (``prediction_type='epsilon'``), 与 NavDP 默认配置一致.
    """
    if scheduler.config.prediction_type != "epsilon":
        raise NotImplementedError(
            f"仅支持 prediction_type='epsilon', 当前={scheduler.config.prediction_type}"
        )

    t = int(timestep)
    # 上一时刻 t-1 (训练时 num_inference_steps == num_train_timesteps)
    prev_t = scheduler.previous_timestep(t)

    alphas_cumprod = scheduler.alphas_cumprod.to(sample.device)
    alpha_prod_t = alphas_cumprod[t]
    alpha_prod_t_prev = alphas_cumprod[prev_t] if prev_t >= 0 else torch.tensor(
        1.0, device=sample.device, dtype=sample.dtype
    )
    beta_prod_t = 1.0 - alpha_prod_t
    beta_prod_t_prev = 1.0 - alpha_prod_t_prev

    current_alpha_t = alpha_prod_t / alpha_prod_t_prev
    current_beta_t = 1.0 - current_alpha_t

    # 1) 由 epsilon 预测 x0
    pred_x0 = (sample - beta_prod_t.sqrt() * model_output) / alpha_prod_t.sqrt()
    if getattr(scheduler.config, "clip_sample", False):
        clip_range = float(getattr(scheduler.config, "clip_sample_range", 1.0))
        pred_x0 = pred_x0.clamp(-clip_range, clip_range)

    # 2) 后验均值 (DDPM 论文 (7) 式)
    coef_x0 = (alpha_prod_t_prev.sqrt() * current_beta_t) / beta_prod_t
    coef_xt = (current_alpha_t.sqrt() * beta_prod_t_prev) / beta_prod_t
    mean = coef_x0 * pred_x0 + coef_xt * sample

    # 3) 后验方差: 与 scheduler._get_variance(t) 一致 (fixed_small_log 等)
    variance_type = getattr(scheduler.config, "variance_type", "fixed_small")
    variance = beta_prod_t_prev / beta_prod_t * current_beta_t  # 默认 fixed_small
    variance = variance.clamp(min=eps)
    if variance_type == "fixed_small_log":
        std = (0.5 * variance.log()).exp()
    elif variance_type == "fixed_large":
        variance = current_beta_t
        std = variance.sqrt()
    else:
        std = variance.sqrt()

    # t==0 步通常确定性 (std=0); 与 diffusers 行为一致
    if prev_t < 0:
        std = std * 0.0

    return {"mean": mean, "std": std, "pred_x0": pred_x0}


def _gaussian_logprob(
    sample: torch.Tensor, mean: torch.Tensor, std: torch.Tensor, eps: float = 1e-12
) -> torch.Tensor:
    """逐元素高斯 log-prob, std 标量或可广播. 当 std==0 (确定性步) 返回 0."""
    std_safe = torch.clamp(std, min=eps)
    # 标量 std 情形 -> 直接计算; 张量情形按广播
    log_norm = -0.5 * ((sample - mean) ** 2) / (std_safe ** 2) - std_safe.log() - 0.5 * math.log(2 * math.pi)
    # 若 std == 0 (最后一步) 视为退化分布, log-prob 置 0 (与 RLinf get_logprob_norm 一致)
    is_zero = (std <= eps)
    if isinstance(is_zero, torch.Tensor) and is_zero.any():
        log_norm = torch.where(is_zero.expand_as(log_norm), torch.zeros_like(log_norm), log_norm)
    return log_norm


# ---------------------------------------------------------------------------
# NavDP GRPO 子类
# ---------------------------------------------------------------------------


class NavDPGRPOModelConfig(NavDPModelConfig):
    model_type = "navdp_grpo"


class NavDPGRPONet(NavDPNet):
    """在 NavDP 基础上提供 GRPO 所需的两个接口.

    新增方法:
        - sample_actions_with_logprob: rollout 阶段使用, 返回 chain & log-prob.
        - compute_logprob_value: 训练阶段使用, 在同一 chain 上重计算 log-prob.
        - predict_noise_grpo: 覆写父类的 predict_noise, 修正 .repeat() 为 repeat_interleave,
          以支持 B>1 的 GRPO batch.

    其他行为 (前向 / SFT 损失) 完全复用 ``NavDPNet``.
    """

    config_class = NavDPGRPOModelConfig

    # ---------------- 覆写 predict_noise ----------------

    def predict_noise_grpo(self, last_actions, timestep, goal_embed, rgbd_embed):
        """修正版 predict_noise, 支持 B>1.

        父类 predict_noise L165 使用 cond_embedding.repeat(action_embeds.shape[0], 1, 1),
        即 tile N 次. 这在 B=1 时 (原始推理代码) 正确: cond [1,...] → repeat(total,...).
        但 B>1 时 (GRPO rollout_batch_size>1): cond [B,...] → repeat(total,...) = [B*total,...] ❌.

        本方法改用 repeat_interleave: 将 [B, seq, dim] 中的每个 prompt 沿 batch 维度
        复制 sample_num=total/B 次, 得到 [total, seq, dim], 与 action_embeds 对齐.
        """
        action_embeds = self.input_embed(last_actions)
        time_embeds = self.time_emb(timestep.to(self._device)).unsqueeze(1)
        cond_input = torch.cat(
            [time_embeds, goal_embed, goal_embed, goal_embed, rgbd_embed], dim=1
        )
        cond_embedding = cond_input + self.cond_pos_embed(cond_input)

        # 关键修正: repeat → repeat_interleave
        # action_embeds.shape[0] == total, cond_embedding.shape[0] == B
        # 需要每个 prompt 复制 total//B 次 (= sample_num 或 group_size)
        repeat_factor = action_embeds.shape[0] // cond_embedding.shape[0]
        cond_embedding = cond_embedding.repeat_interleave(repeat_factor, dim=0)

        input_embedding = action_embeds + self.out_pos_embed(action_embeds)
        output = self.decoder(tgt=input_embedding, memory=cond_embedding, tgt_mask=self.tgt_mask.to(self._device))
        output = self.layernorm(output)
        output = self.action_head(output)
        return output

    # ---------------- 条件 embedding 抽取 ----------------

    def _encode_conditions(
        self,
        goal_point: Optional[torch.Tensor],
        goal_image: Optional[torch.Tensor],
        goal_pixel: Optional[torch.Tensor],
        input_images: torch.Tensor,
        input_depths: torch.Tensor,
        goal_type: str = "point",
    ) -> Dict[str, torch.Tensor]:
        """根据 goal_type 选择条件分支, 输出 ``goal_embed`` 与 ``rgbd_embed``.

        goal_type: ``point`` / ``image`` / ``pixel`` / ``nogoal``.
        与 NavDPNet.predict_pointgoal_batch_action_vel 等推理函数等价.
        """
        rgbd_embed = self.rgbd_encoder(input_images, input_depths)
        if goal_type == "point":
            assert goal_point is not None
            tensor_point_goal = torch.as_tensor(goal_point, dtype=torch.float32, device=self._device)
            goal_embed = self.point_encoder(tensor_point_goal).unsqueeze(1)
        elif goal_type == "image":
            goal_embed = self.image_encoder(goal_image).unsqueeze(1)
        elif goal_type == "pixel":
            goal_embed = self.pixel_encoder(goal_pixel).unsqueeze(1)
        elif goal_type == "nogoal":
            goal_embed = torch.zeros_like(rgbd_embed[:, 0:1])
        else:
            raise ValueError(f"未知 goal_type=`{goal_type}`")
        return {"goal_embed": goal_embed, "rgbd_embed": rgbd_embed}

    # ---------------- 采样: 带 log-prob 的 DDPM 反向过程 ----------------

    @torch.no_grad()
    def sample_actions_with_logprob(
        self,
        input_images: torch.Tensor,
        input_depths: torch.Tensor,
        goal_point: Optional[torch.Tensor] = None,
        goal_image: Optional[torch.Tensor] = None,
        goal_pixel: Optional[torch.Tensor] = None,
        goal_type: str = "point",
        sample_num: int = 1,
        num_inference_steps: Optional[int] = None,
        temperature: float = 1.0,
    ) -> Dict[str, Any]:
        """DDPM 反向采样并保存高斯 log-prob.

        Args:
            input_images / input_depths: ``[B, memory_size, ...]`` 观测.
            goal_*: 三选一目标输入.
            goal_type: 选择哪一种 goal 分支.
            sample_num: 每个 batch 元素并行采样的轨迹数 (用于 GRPO group).
            num_inference_steps: 默认与 ``self.noise_scheduler.config.num_train_timesteps`` 相同
                (NavDP 默认为 10). 调小可加速但会引入 ODE 误差.
            temperature: 采样温度系数. 每步DDPM计算出后验mean/std后, 将std乘以T再加噪声;
                >1.0 放大后验方差σₜ×T提高探索度, <1.0降低随机性.
                log-prob使用effective_std(σₜ×T)计算, 与采样分布一致,
                temperature在ratio分子分母中抵消, ratio只反映策略参数变化.

        Returns:
            dict: 包含
                - ``actions``: ``[B*sample_num, predict_size, 3]`` 最终样本.
                - ``chains``:  ``[T+1, B*sample_num, predict_size, 3]`` 每个反向步样本.
                - ``timesteps``: ``[T]`` (long).
                - ``prev_logprobs``: ``[T, B*sample_num]`` (per-step joint log-prob, 已对动作维求和).
                - ``prev_logprobs_elem``: ``[T, B*sample_num, predict_size, 3]`` 逐元素 log-prob.
                - ``denoise_inds``: 与 ``prev_logprobs`` 形状一致, 值为对应 timestep, 便于训练 mini-batch.
                - ``temperature``: 采样时使用的 temperature 值 (用于训练时 log-prob 重算).
        """
        if num_inference_steps is None:
            num_inference_steps = self.noise_scheduler.config.num_train_timesteps

        cond = self._encode_conditions(
            goal_point, goal_image, goal_pixel, input_images, input_depths, goal_type=goal_type
        )
        goal_embed = cond["goal_embed"]
        rgbd_embed = cond["rgbd_embed"]
        batch = goal_embed.shape[0]
        total = batch * sample_num

        # 初始噪声 x_T ~ N(0, I)
        x = torch.randn((total, self.predict_size, 3), device=self._device)

        self.noise_scheduler.set_timesteps(num_inference_steps)
        timesteps = self.noise_scheduler.timesteps.to(self._device)
        T = timesteps.shape[0]

        chains = [x.detach().clone()]
        prev_logprobs_elem: List[torch.Tensor] = []

        for k, t in enumerate(timesteps):
            # predict_noise_grpo 要求 timestep 的 batch 维度与 goal_embed / rgbd_embed
            # 一致 (因为 torch.cat 在 dim=1 上要求 batch 对齐).
            # goal_embed [B,...], timestep 也要 [B], rgbd_embed [B,...].
            # predict_noise_grpo 内部用 repeat_interleave 将 [B,...] 扩展到 [total,...].
            t_tensor = t.unsqueeze(0).expand(batch).to(self._device)
            noise_pred = self.predict_noise_grpo(
                last_actions=x,
                timestep=t_tensor,
                goal_embed=goal_embed,
                rgbd_embed=rgbd_embed,
            )
            params = _ddpm_posterior_params(
                self.noise_scheduler, model_output=noise_pred, timestep=int(t.item()), sample=x
            )
            mean = params["mean"]
            std = params["std"]
            # 采样 x_{t-1}: 在加噪声前将后验方差 σₜ 乘以 temperature
            # temperature > 1.0: 放大去噪噪声幅度, 让生成轨迹更随机/跳得更远 (提高探索度)
            # temperature < 1.0: 缩小噪声幅度, 降低随机性
            effective_std = std * temperature
            if torch.is_tensor(effective_std) and float(effective_std.max().item()) > 0.0:
                noise = torch.randn_like(x)
                x_prev = mean + effective_std * noise
            else:
                x_prev = mean

            # log-prob 必须使用 effective_std 计算, 与采样分布 N(mean, (σ×T)²I) 一致
            # 否则 x_prev 偏离 mean 的程度比原始 σ 预期更大, log-prob 偏小, ratio 爆炸
            logp_elem = _gaussian_logprob(x_prev, mean, effective_std)
            prev_logprobs_elem.append(logp_elem)
            chains.append(x_prev.detach().clone())
            x = x_prev

        chains_tensor = torch.stack(chains, dim=0)  # [T+1, total, pred, 3]
        logp_elem = torch.stack(prev_logprobs_elem, dim=0)  # [T, total, pred, 3]
        # joint log-prob: 对 action 维度求和 -> [T, total]
        joint_logp = logp_elem.flatten(2).sum(dim=-1)
        denoise_inds = timesteps.view(T, 1).expand(T, total).clone()

        return {
            "actions": chains_tensor[-1],
            "chains": chains_tensor,
            "timesteps": timesteps,
            "prev_logprobs": joint_logp,
            "prev_logprobs_elem": logp_elem,
            "denoise_inds": denoise_inds,
            # 原始 B 大小的条件 embedding (predict_noise 内部自动 repeat)
            "goal_embed": goal_embed.detach(),
            "rgbd_embed": rgbd_embed.detach(),
            # group_id: total 中的每条轨迹属于 B 中的哪个 prompt (用于 micro-batch 索引)
            "group_ids": torch.arange(batch, device=self._device).repeat_interleave(sample_num),
            # 采样 temperature (记录, 不再用于 compute_logprob_value)
            "temperature": temperature,
        }

    # ---------------- 训练: 在固定 chain 上重计算 log-prob ----------------

    def compute_logprob_value(
        self,
        chains: torch.Tensor,
        timesteps: torch.Tensor,
        goal_embed: torch.Tensor,
        rgbd_embed: torch.Tensor,
        step_indices: Optional[torch.Tensor] = None,
        joint_logprob: bool = True,
        temperature: float = 1.0,
    ) -> Dict[str, torch.Tensor]:
        """在与采样阶段相同的 ``chains`` 上重新计算当前策略下的 log-prob.

        log-prob 使用 effective_std (σₜ×T) 计算, 与采样分布一致.
        prev_logprobs 和 new logprobs 在同一分布下计算 ratio,
        temperature 在分子分母中抵消, ratio 只反映策略参数变化.

        Args:
            chains: ``[T+1, total, predict_size, 3]`` (来自 sample_actions_with_logprob).
            timesteps: ``[T]`` (long), 与采样时一致.
            goal_embed / rgbd_embed: 条件 embedding (建议直接复用 rollout 时的结果).
            step_indices: 可选 ``[K]`` 子集, 训练时只对部分时间步求导以省显存.
            joint_logprob: True -> 返回每步对动作维求和的 joint log-prob ``[T_used, total]``;
                False -> 返回逐元素 log-prob ``[T_used, total, pred, 3]``.
            temperature: 采样时使用的温度系数, 需与 rollout 时的 temperature 一致.

        Returns:
            dict: {"logprobs": Tensor, "logprobs_elem": Tensor, "step_indices": LongTensor}.
        """
        T = timesteps.shape[0]
        if step_indices is None:
            step_indices = torch.arange(T, device=timesteps.device)

        logp_elem_list: List[torch.Tensor] = []
        for k in step_indices.tolist():
            t = timesteps[k]
            x_t = chains[k]  # 与采样同一个张量
            x_prev = chains[k + 1]
            # timestep batch 维度必须与 goal_embed / rgbd_embed 一致
            mb = goal_embed.shape[0]
            t_tensor = t.unsqueeze(0).expand(mb).to(self._device)
            noise_pred = self.predict_noise_grpo(
                last_actions=x_t,
                timestep=t_tensor,
                goal_embed=goal_embed,
                rgbd_embed=rgbd_embed,
            )
            params = _ddpm_posterior_params(
                self.noise_scheduler, model_output=noise_pred, timestep=int(t.item()), sample=x_t
            )
            # log-prob 使用 effective_std (σₜ×T), 与采样分布一致
            # prev_logprobs 和 new logprobs 在同一分布下计算, ratio 才正常
            effective_std = params["std"] * temperature
            logp = _gaussian_logprob(x_prev, params["mean"], effective_std)
            logp_elem_list.append(logp)

        logp_elem = torch.stack(logp_elem_list, dim=0)  # [K, total, pred, 3]
        joint_logp = logp_elem.flatten(2).sum(dim=-1)

        return {
            "logprobs": joint_logp if joint_logprob else logp_elem,
            "logprobs_elem": logp_elem,
            "step_indices": step_indices.to(torch.long),
        }

    # ---------------- Critic 辅助损失 (方案 B) ----------------

    def predict_critic_grpo(self, predict_trajectory, rgbd_embed):
        """修正版 predict_critic, 支持 B>1.

        父类 predict_critic L173 使用 rgbd_embed.repeat(predict_trajectory.shape[0], 1, 1),
        即 tile N 次. 与 predict_noise 同理, B>1 时会错乱.

        本方法改用 repeat_interleave: 将 [B, seq, dim] 中的每个 prompt 沿 batch 维度
        复制 repeat_factor 次, 得到 [total, seq, dim], 与 predict_trajectory 对齐.

        与 SFT forward 中 critic 路径保持一致:
        - input_embed 输出 detach (不将 critic 梯度传回 input_embed)
        - 条件构造: [nogoal×4, rgbd_embed] + cond_pos_embed
        - decoder 使用 memory_mask=self.cond_critic_mask
        """
        repeat_factor = predict_trajectory.shape[0] // rgbd_embed.shape[0]
        repeat_rgbd_embed = rgbd_embed.repeat_interleave(repeat_factor, dim=0)
        nogoal_embed = torch.zeros_like(repeat_rgbd_embed[:, 0:1])
        # 与 SFT forward L209-210 一致: input_embed 输出 detach
        action_embeddings = self.input_embed(predict_trajectory).detach()
        action_embeddings = action_embeddings + self.out_pos_embed(action_embeddings)
        cond_input = torch.cat(
            [nogoal_embed, nogoal_embed, nogoal_embed, nogoal_embed, repeat_rgbd_embed], dim=1
        )
        cond_embeddings = cond_input + self.cond_pos_embed(cond_input)
        critic_output = self.decoder(
            tgt=action_embeddings, memory=cond_embeddings, memory_mask=self.cond_critic_mask.to(self._device)
        )
        critic_output = self.layernorm(critic_output)
        critic_output = self.critic_head(critic_output.mean(dim=1))[:, 0]
        return critic_output

    def compute_critic_loss(
        self,
        actions: torch.Tensor,
        rgbd_embed: torch.Tensor,
        group_ids: torch.Tensor,
        batch_label_critic: torch.Tensor,
        batch_augment_critic: Optional[torch.Tensor] = None,
        use_augment: bool = True,
    ) -> Dict[str, torch.Tensor]:
        """SFT critic 辅助损失, 在 GRPO 更新步中与 PPO actor loss 联合反传.

        与 NavDPTrainer.compute_loss 中 critic_loss 的计算方式一致:
            critic_loss = (critic_pred - label_critic).square().mean()
                        + (augment_pred - augment_critic).square().mean()

        与 SFT forward 中 critic 路径的对齐:
        - input_embed 输出 detach (predict_critic_grpo 内部): 梯度仅流经
          decoder → layernorm → critic_head, 不回传到 input_embed
        - 条件构造: [nogoal×4, rgbd_embed] + cond_pos_embed + memory_mask
        - loss 计算: MSE (square().mean())

        与 SFT 的差异:
        - 输入轨迹: SFT 用 GT labels/augments, GRPO 用 rollout 采样的 actions
          (GRPO 训练时没有 GT action labels, 只能对采样轨迹做 critic 监督)
        - augment 分支: SFT 中 label 和 augment 是不同轨迹, GRPO 中两者共用
          同一条 rollout action (因 GRPO 无 augment trajectory)
        """
        # 用 group_ids 映射到原始 B 的 prompt 索引
        label_critic = batch_label_critic[group_ids]  # [total]
        if use_augment and batch_augment_critic is not None:
            augment_critic = batch_augment_critic[group_ids]  # [total]

        # critic 前向: predict_critic_grpo 自动处理 B>1 的 repeat
        # 内部 input_embed.detach() 保证梯度路径与 SFT 一致
        critic_pred = self.predict_critic_grpo(actions, rgbd_embed)  # [total]

        # label critic loss
        critic_label_loss = (critic_pred - label_critic).square().mean()

        # augment critic loss (可选)
        # SFT 阶段 cr_augment_pred 用 augment trajectory, cr_label_pred 用 label trajectory.
        # GRPO 阶段无 augment trajectory, 因此共用 rollout actions.
        # 由于 predict_critic_grpo 内部 input_embed.detach(), 同一 actions 两次前向
        # 结果相同, 直接复用 critic_pred.
        if use_augment and batch_augment_critic is not None:
            critic_augment_loss = (critic_pred - augment_critic).square().mean()
        else:
            critic_augment_loss = torch.tensor(0.0, device=actions.device)

        critic_loss = critic_label_loss + critic_augment_loss

        return {
            "critic_loss": critic_loss,
            "critic_label_loss": critic_label_loss,
            "critic_augment_loss": critic_augment_loss,
        }

    # ---------------- 与父类一致的 from_pretrained 兼容 ----------------

    @classmethod
    def from_pretrained(cls, pretrained_model_name_or_path, *model_args, **kwargs):
        # 复用 NavDPNet.from_pretrained 的逻辑, 但配置类替换为 GRPO 子类.
        config = kwargs.pop("config", None)
        if config is None:
            config = cls.config_class.from_pretrained(pretrained_model_name_or_path, **kwargs)
        if hasattr(config, "model_dump"):
            config = cls.config_class(model_cfg=config)

        model = cls(config)
        model.to(model._device)

        if pretrained_model_name_or_path and os.path.isdir(pretrained_model_name_or_path):
            # 支持多种checkpoint文件名: pytorch_model.bin 或 navdp.ckpt
            pytorch_model_path = os.path.join(pretrained_model_name_or_path, "pytorch_model.bin")
            navdp_ckpt_path = os.path.join(pretrained_model_name_or_path, "navdp.ckpt")
            
            if os.path.exists(pytorch_model_path):
                ckpt_path = pytorch_model_path
            elif os.path.exists(navdp_ckpt_path):
                ckpt_path = navdp_ckpt_path
            else:
                raise FileNotFoundError(
                    f"No checkpoint found in {pretrained_model_name_or_path}. "
                    f"Expected 'pytorch_model.bin' or 'navdp.ckpt'"
                )
            
            print(f"Loading checkpoint from: {ckpt_path}")
            incompatible_keys, _ = model.load_state_dict(
                torch.load(ckpt_path, map_location='cpu', weights_only=False), strict=False
            )
            if incompatible_keys:
                print(f"Incompatible keys: {incompatible_keys}")
        elif pretrained_model_name_or_path:
            incompatible_keys, _ = model.load_state_dict(
                torch.load(pretrained_model_name_or_path, map_location='cpu', weights_only=False), strict=False
            )
            if incompatible_keys:
                print(f"Incompatible keys: {incompatible_keys}")

        return model