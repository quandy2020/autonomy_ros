"""轨迹相关公共工具.

将 NavDP 模型输出的 actions [T, >=2] 还原为：
    - waypoints: 累积位移 (米, 起点 (0,0))
    - delta_xy:  逐步物理位移 (米)
    - speeds:    逐步真实速度 (m/s)

所有奖励函数（target_speed / smooth / terminal_heading / progress / success
应共用本模块, 避免在各文件重复实现 actions->waypoints 逻辑、且公式不一致.

NavDP 数据集中 actions 的归一化约定:
    actions[:, :2] = 单步真实位移(米) × action_scale (默认 4.0)

单步真实时间:
    dt = frame_dt × sample_interval

其中 frame_dt 是数据集**原始采样周期** (秒, 与采集帧率挂钩, 不是控制频率).
"""

from __future__ import annotations

from typing import Optional, Tuple, Union

import numpy as np
import torch

ArrayLike = Union[np.ndarray, torch.Tensor, list, tuple]


def _to_numpy(actions: ArrayLike) -> np.ndarray:
    if isinstance(actions, torch.Tensor):
        return actions.detach().cpu().numpy()
    return np.asarray(actions)


def _validate_actions(actions: np.ndarray) -> np.ndarray:
    if actions.size == 0:
        raise ValueError("actions 不能为空数组")
    if actions.ndim != 2:
        raise ValueError(
            f"actions 必须是 2D 数组 [T, >=2], 当前 ndim={actions.ndim}, shape={actions.shape}"
        )
    if actions.shape[-1] < 2:
        raise ValueError(
            f"actions 至少需要 2 个维度 (dx, dy), 当前 shape={actions.shape}"
        )
    return actions


def actions_to_delta_xy(
    actions: ArrayLike,
    action_scale: float = 4.0,
) -> np.ndarray:
    """从 actions 还原逐步物理位移 (米).

    Args:
        actions: [T, >=2], NavDP 输出动作 (已 ×action_scale).
        action_scale: 反归一化因子, 默认 4.0.

    Returns:
        np.ndarray: [T, 2] (dx, dy) 单位 米.
    """
    if action_scale <= 0:
        raise ValueError(f"action_scale 必须为正数, 当前={action_scale}")
    arr = _validate_actions(_to_numpy(actions))
    return arr[:, :2].astype(np.float64) / float(action_scale)


def actions_to_waypoints(
    actions: ArrayLike,
    action_scale: float = 4.0,
    start_xy: Optional[ArrayLike] = None,
) -> np.ndarray:
    """从 actions 累积得到 waypoints (米, 起点默认 (0,0)).

    Args:
        actions: [T, >=2].
        action_scale: 反归一化因子.
        start_xy: 可选 [2], 偏移起点; 若为 None 则起点 (0,0).

    Returns:
        np.ndarray: [T, 2] 累积位置.
    """
    delta_xy = actions_to_delta_xy(actions, action_scale=action_scale)
    waypoints = np.cumsum(delta_xy, axis=0)
    if start_xy is not None:
        s = _to_numpy(start_xy).astype(np.float64).reshape(-1)[:2]
        waypoints = waypoints + s
    return waypoints


def actions_to_waypoints_with_start(
    actions: ArrayLike,
    action_scale: float = 4.0,
    start_xy: Optional[ArrayLike] = None,
) -> np.ndarray:
    """返回显式包含起点的轨迹点序列 [T+1, 2]."""
    start = (
        np.zeros(2, dtype=np.float64)
        if start_xy is None
        else _to_numpy(start_xy).astype(np.float64).reshape(-1)[:2]
    )
    waypoints = actions_to_waypoints(
        actions,
        action_scale=action_scale,
        start_xy=start_xy,
    )
    return np.vstack([start, waypoints])


def ensure_waypoints_include_start(
    waypoints: ArrayLike,
    start_xy: Optional[ArrayLike] = None,
    atol: float = 1e-8,
) -> np.ndarray:
    """确保轨迹首点显式包含起点.

    统一 actions/waypoints 两种输入路径的语义:
    - actions_to_waypoints 返回的是未来累计点, 不包含起点
    - 某些外部直传的 waypoints 可能已经包含起点, 也可能不包含
    """
    wp = _to_numpy(waypoints).astype(np.float64)
    if wp.ndim != 2 or wp.shape[-1] < 2:
        raise ValueError(f"waypoints 必须形如 [T, >=2], 当前 shape={wp.shape}")
    start = (
        np.zeros(2, dtype=np.float64)
        if start_xy is None
        else _to_numpy(start_xy).astype(np.float64).reshape(-1)[:2]
    )
    if wp.shape[0] == 0:
        return start.reshape(1, 2)
    first_xy = wp[0, :2]
    if np.allclose(first_xy, start, atol=atol, rtol=0.0):
        return wp[:, :2]
    return np.vstack([start, wp[:, :2]])


def compute_speeds(
    actions: ArrayLike,
    action_scale: float = 4.0,
    dt: float = 1.0 / 30.0 * 4,
) -> np.ndarray:
    """计算逐步真实速度 (m/s).

    speed[i] = ||delta_xy[i]||_2 / dt

    Args:
        actions: [T, >=2].
        action_scale: 反归一化因子.
        dt: 单步真实时间间隔 (秒) = frame_dt × sample_interval.

    Returns:
        np.ndarray: [T] 速度 (m/s).
    """
    if dt <= 0:
        raise ValueError(f"dt 必须为正数, 当前={dt}")
    delta_xy = actions_to_delta_xy(actions, action_scale=action_scale)
    return np.linalg.norm(delta_xy, axis=-1) / float(dt)


def actions_to_trajectory_bundle(
    actions: ArrayLike,
    action_scale: float = 4.0,
    dt: float = 1.0 / 30.0 * 4,
    start_xy: Optional[ArrayLike] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """一次性返回 (waypoints, delta_xy, speeds).

    供奖励函数复用: 三者共享同一份 delta_xy 计算, 避免数值不一致.

    Returns:
        waypoints: [T, 2] 累积位置 (米).
        delta_xy:  [T, 2] 逐步位移  (米).
        speeds:    [T]    逐步速度 (m/s).
    """
    if dt <= 0:
        raise ValueError(f"dt 必须为正数, 当前={dt}")
    delta_xy = actions_to_delta_xy(actions, action_scale=action_scale)
    waypoints = np.cumsum(delta_xy, axis=0)
    if start_xy is not None:
        s = _to_numpy(start_xy).astype(np.float64).reshape(-1)[:2]
        waypoints = waypoints + s
    speeds = np.linalg.norm(delta_xy, axis=-1) / float(dt)
    return waypoints, delta_xy, speeds


def derive_dt(frame_dt: float, sample_interval: int) -> float:
    """dt = frame_dt × sample_interval, 带参数合法性校验."""
    if frame_dt <= 0:
        raise ValueError(f"frame_dt 必须为正数, 当前={frame_dt}")
    if sample_interval <= 0:
        raise ValueError(f"sample_interval 必须为正整数, 当前={sample_interval}")
    return float(frame_dt) * float(sample_interval)


def derive_success_effective_threshold(
    target_speed: float,
    frame_dt: float,
    sample_interval: int,
    horizon_points: int = 24,
) -> float:
    """成功奖励生效阈值 = target_speed × frame_dt × sample_interval × horizon_points."""
    if target_speed < 0:
        raise ValueError(f"target_speed 必须 >= 0, 当前={target_speed}")
    if horizon_points <= 0:
        raise ValueError(f"horizon_points 必须为正整数, 当前={horizon_points}")
    return float(target_speed) * derive_dt(frame_dt, sample_interval) * float(horizon_points)
