"""限速奖励 (SpeedLimitRewardFn).

逐点计算输出轨迹每个点的真实物理速度（m/s），对超过速度上限的点进行软惩罚累加。

惩罚规则（软惩罚，上不封顶）：
    - 速度 <= speed_limit (默认 1 m/s): 惩罚 0
    - 速度 > speed_limit: 起步惩罚 (1/predict_size)，每超 1 m/s 再加 (12/predict_size)
      即: penalty_per_point = (1/predict_size) + (v - speed_limit) * (12/predict_size)
      （当 v > speed_limit 时单点惩罚从 1/24 起步，每多 1 m/s 增加 12/24 ≈ 0.5）

输入: trajectories[i] 必须包含
    - ``actions``: [predict_size, 3] (dx, dy, dθ) 增量, 已 ×4 (NavDP 模型输出约定);

输出: [batch_size] float tensor, 各轨迹累计惩罚（>=0，可大于 1）。

真实速度计算（重要！）:
-------------------------------------------------------------------
NavDP 数据集中 actions 的归一化/反归一流程:
    1. 数据集内:  pred_actions = (raw_xy[1:] - raw_xy[:-1]) * 4.0
       即 actions 中每分量 = 单步物理位移(米) × 4
    2. 还原物理位移: delta_xy = actions[:, :2] / 4.0   (单位: 米)
    3. 单步真实时间: dt = frame_dt × sample_interval   (单位: 秒)
       - frame_dt: **数据集原始采样周期** (秒, 由数据采集时的帧率决定),
         注意: 这里指的是数据集"专家轨迹被记录的周期", 而不是模型部署时的
         控制周期. 例如:
           * NavDP 30Hz 原始数据集 → frame_dt = 1/30 ≈ 0.0333s
           * 10Hz 数据集            → frame_dt = 0.1s
       - sample_interval: 数据集采样间隔, 与训练/评测时数据集一致
       例: 30Hz 数据 + sample_interval=4 → dt = (1/30) × 4 ≈ 0.1333s
    4. 真实速度: speed = ||delta_xy|| / dt              (单位: 米/秒)

⚠️ 关键: 训练/评测时必须确保 frame_dt 与数据集原始帧率一致, sample_interval
   与数据集实际使用的 sample_interval 一致, 否则真实速度计算会出错。
   本模块通过 config 传入 frame_dt + sample_interval 计算 dt, 避免手动设置
   dt 导致的物理含义错位。
"""

from typing import Any, Dict, List, Optional

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn


def compute_speed_limit_reward(
    actions: np.ndarray,
    speed_limit: float = 1.0,
    predict_size: Optional[int] = None,
    action_scale: float = 4.0,
    dt: float = 0.1333333,
    base_penalty_ratio: float = 1.0,
    step_penalty_ratio: float = 12.0,
    eps: float = 1e-6,
) -> float:
    """计算单条轨迹的限速软惩罚。

    Args:
        actions: shape=[T, >=2], 模型输出的动作序列 (已 ×action_scale 缩放).
            actions[:, :2] 表示 (dx, dy) 增量, 单位 = 米 × action_scale.
        speed_limit: 真实速度上限 (米/秒), 默认 1.0.
        predict_size: 单点惩罚的归一化分母, 默认取 actions 长度 T.
        action_scale: 反归一化因子, 默认 4.0 (NavDP 约定).
        dt: 单步对应的真实时间间隔(秒). 由 frame_dt × sample_interval 算出,
            默认 1/30 × 4 ≈ 0.1333s (即 30Hz 数据集 × sample_interval=4).
        base_penalty_ratio: 超速起步惩罚倍数, 默认 1.0 (即 1/predict_size).
        step_penalty_ratio: 每超 1 m/s 增量倍数, 默认 12.0 (即 12/predict_size).
        eps: 浮点数容差, 默认 1e-6, 用于避免边界浮点误差误判为超速.

    Returns:
        float: 累计惩罚值 (>=0, 上不封顶).
    """
    # 兼容 list / tuple / torch.Tensor 输入: 统一转换为 numpy ndarray
    if isinstance(actions, torch.Tensor):
        actions = actions.detach().cpu().numpy()
    else:
        actions = np.asarray(actions)

    if actions.size == 0:
        raise ValueError("actions 不能为空数组")

    # 必须是 2D: [T, >=2], 其中 [:, :2] 表示 (dx, dy)
    if actions.ndim != 2:
        raise ValueError(
            f"actions 必须是 2D 数组 [T, >=2], 当前 ndim={actions.ndim}, shape={actions.shape}"
        )

    if actions.shape[-1] < 2:
        raise ValueError(
            f"actions 至少需要 2 个维度 (dx, dy), 当前 shape={actions.shape}"
        )

    T = actions.shape[0]
    if predict_size is None or predict_size <= 0:
        predict_size = T

    if action_scale <= 0:
        raise ValueError(f"action_scale 必须为正数, 当前={action_scale}")
    if dt <= 0:
        raise ValueError(f"dt 必须为正数, 当前={dt}")

    # 1. 还原物理位移 (单位: 米)
    delta_xy = actions[:, :2].astype(np.float64) / float(action_scale)

    # 2. 真实速度 (单位: 米/秒)
    speeds = np.linalg.norm(delta_xy, axis=-1) / float(dt)  # [T]

    # 3. 超速判定与超速量 (>0 表示超速). 使用 eps 容差避免浮点边界误判.
    threshold = float(speed_limit) + float(eps)
    overspeed_mask = (speeds > threshold).astype(np.float64)
    overspeed = np.maximum(speeds - float(speed_limit), 0.0)

    # 4. 软惩罚: 超速点 = base * 1/N + step * overspeed * 1/N
    #    超速点惩罚 = (1/predict_size) + overspeed * (12/predict_size)
    per_point_unit = 1.0 / float(predict_size)
    penalties = overspeed_mask * (
        base_penalty_ratio * per_point_unit
        + step_penalty_ratio * per_point_unit * overspeed
    )

    return float(penalties.sum())


@register_reward_fn("speed_limit")
class SpeedLimitRewardFn(RewardFn):
    """限速奖励函数 (惩罚项, 上不封顶, 真实物理速度 m/s).

    config 字段:
        speed_limit: 真实速度上限 (m/s), 默认 1.0.
        frame_dt: **数据集原始采样周期** (秒), 默认 1/30 ≈ 0.0333 (30Hz 数据集).
            注意: 这里指数据采集时的帧率, 不是模型部署的控制频率.
        sample_interval: 数据集采样间隔, 默认 4. 必须与训练/评测数据集一致.
        action_scale: 反归一化因子, 默认 4.0 (NavDP 约定).
        base_penalty_ratio: 超速起步惩罚倍数, 默认 1.0 (即 1/predict_size).
        step_penalty_ratio: 每超 1 m/s 增量倍数, 默认 12.0 (即 12/predict_size).

    自动推导: dt = frame_dt × sample_interval
        默认: dt = (1/30) × 4 ≈ 0.1333 秒
    """

    name: str = "speed_limit"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.speed_limit = float(self.config.get("speed_limit", 1.0))
        self.action_scale = float(self.config.get("action_scale", 4.0))
        # 默认 1/30 (30Hz 数据集), 与 NavDP 原始训练数据采样率一致
        self.frame_dt = float(self.config.get("frame_dt", 1.0 / 30.0))
        self.sample_interval = int(self.config.get("sample_interval", 4))
        if self.frame_dt <= 0:
            raise ValueError(f"frame_dt 必须为正数, 当前={self.frame_dt}")
        if self.sample_interval <= 0:
            raise ValueError(
                f"sample_interval 必须为正整数, 当前={self.sample_interval}"
            )
        # 真实单步时间间隔
        self.dt = self.frame_dt * float(self.sample_interval)
        self.base_penalty_ratio = float(self.config.get("base_penalty_ratio", 1.0))
        self.step_penalty_ratio = float(self.config.get("step_penalty_ratio", 12.0))

    def compute(
        self,
        trajectories: List[Dict[str, Any]],
        **kwargs: Any,
    ) -> torch.Tensor:
        """计算 batch reward.

        Args:
            trajectories: rollout 轨迹列表, 每个元素必须包含 actions.
            **kwargs: device, dtype 等.

        Returns:
            torch.Tensor: 形状 [batch_size] 的 float reward (>=0, 上不封顶).
        """
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

        rewards: List[float] = []
        for traj_idx, traj in enumerate(trajectories):
            actions = traj.get("actions")
            if actions is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 actions, 无法计算限速奖励"
                )

            if isinstance(actions, torch.Tensor):
                actions_np = actions.detach().cpu().numpy()
            else:
                actions_np = np.asarray(actions)

            reward = compute_speed_limit_reward(
                actions_np,
                speed_limit=self.speed_limit,
                predict_size=actions_np.shape[0],
                action_scale=self.action_scale,
                dt=self.dt,
                base_penalty_ratio=self.base_penalty_ratio,
                step_penalty_ratio=self.step_penalty_ratio,
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)