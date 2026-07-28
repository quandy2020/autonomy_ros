"""占用软惩罚奖励 (OccupancyRewardFn).

使用膨胀障碍物点云（蓝色点 [0, 0, 0.5]）进行占用惩罚计算。

惩罚逻辑：
    - 膨胀障碍物点云已包含膨胀半径（约0.30m），这是障碍物向外膨胀的距离
    - 机器人有实际半径（约0.25m）
    - 膨胀半径 > 机器人半径，差值约为 0.05m
    - collision_threshold = 0.05m = (膨胀半径 - 机器人半径)，表示质点接触膨胀障碍物表面
    - max_threshold = 0.2m 表示安全距离边界

线性惩罚: 对每个 waypoint，计算它到最近障碍物的欧式距离，
- 若距离 < collision_threshold (0.05m)：惩罚 1/predict_size
- 若距离在 [collision_threshold, max_threshold) 内：线性惩罚 = (max_threshold - dist) / (max_threshold - collision_threshold) * (1/predict_size)
- 若距离 >= max_threshold (0.2m)：惩罚为 0

每个点的最大惩罚为 1/predict_size，最终累加所有 waypoint 的惩罚作为占据奖励。

输入: trajectories[i] 必须包含
    - ``actions``: [predict_size, 3] (dx, dy, dθ) 增量, 已 ×4 (NavDP 模型输出约定);
    - ``inflation_points``: [N, 2] 局部机器人坐标系下的膨胀障碍点 (xy) - 推荐，已转换好;
    - ``world_inflation_points``: [M, 2] or [M, 3] 世界坐标系下的膨胀障碍点;
    - ``base_frame_extrinsic``: [4, 4] 起点时刻的相机外参;
    - ``base_extrinsic``: [4, 4] 机器人基座外参.

输出: [batch_size] float tensor, 值为累加线性惩罚 (0~1).

坐标系说明:
    - waypoints 是局部坐标系下的累积位移，起点为 (0, 0)
    - 膨胀障碍物需要转换到同一局部坐标系下进行检测

优先级:
    1. inflation_points (推荐，各 reward 共享，避免重复计算)
    2. world_inflation_points + base_frame_extrinsic + base_extrinsic (使用外参转换)
    3. world_inflation_points + start_world_pos (使用起点位置转换)

GRPO 在外层会按 group 做归一化, 因此绝对量级不敏感, 只要组内有方差即可
驱动 advantage.
"""

from typing import Any, Dict, List, Optional

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .obstacle_sampler import ObstacleSampler
from .trajectory_utils import actions_to_waypoints


def compute_occupancy_reward(
    trajectory: np.ndarray,
    occ_grid: np.ndarray,
    resolution: float,
    origin: np.ndarray,
) -> float:
    """计算单条轨迹的占用软惩罚。

    将轨迹点映射到二值占用栅格上，统计落在障碍物区域的点占比。

    Args:
        trajectory: shape=[H, 2], 轨迹点序列 (x, y).
        occ_grid: shape=[H_grid, W_grid], 二值占用网格 (True=障碍物).
        resolution: 栅格分辨率（米/格）.
        origin: shape=[2], 栅格原点坐标 (x, y).

    Returns:
        float: 落在障碍物的点数 / 总点数 (0~1), 越界点不计入统计.
    """
    # 边界情况检查
    if trajectory.size == 0:
        raise ValueError("trajectory 不能为空数组")

    H = len(trajectory)
    occupied_count = 0
    valid_count = 0

    grid_h, grid_w = occ_grid.shape

    for point in trajectory:
        # 世界坐标转栅格坐标
        grid_x = int((point[0] - origin[0]) / resolution)
        grid_y = int((point[1] - origin[1]) / resolution)

        # 检查是否越界
        if grid_x < 0 or grid_x >= grid_w or grid_y < 0 or grid_y >= grid_h:
            continue  # 越界点不计入统计

        valid_count += 1
        if occ_grid[grid_y, grid_x]:
            occupied_count += 1

    # 全部点都越界的情况
    if valid_count == 0:
        return 0.0

    return occupied_count / valid_count


def compute_occupancy_reward_from_obstacles(
    waypoints: np.ndarray,
    inflation_points: np.ndarray,
    predict_size: int = 24,
) -> float:
    """基于膨胀障碍物点云计算占用线性惩罚。

    注意：将机器人位置视为质点，不考虑机器人半径。
    膨胀障碍物点云已经包含了安全距离的膨胀。

    惩罚规则（每个点独立计算，无阻断逻辑）：
    1. 若距离 < 0.05m：惩罚 1/predict_size
    2. 若距离在 [0.05m, 0.2m) 内：线性惩罚 = (0.2 - dist) / 0.15 * (1/predict_size)
    3. 若距离 >= 0.2m：惩罚为 0

    Args:
        waypoints: shape=[H, 2], 轨迹点序列 (x, y), 局部坐标系.
        inflation_points: shape=[N, 2], 膨胀障碍物点 (x, y), 局部坐标系.
        predict_size: 轨迹预测步数，默认 24.

    Returns:
        float: 累加惩罚值, 范围 [0, 1] (最多 predict_size * 1/predict_size = 1).
    """
    if waypoints.size == 0:
        raise ValueError("waypoints 不能为空数组")

    if inflation_points.size == 0 or len(inflation_points) == 0:
        return 0.0

    # 过滤填充点 (1e6)
    valid_mask = np.abs(inflation_points).max(axis=-1) < 1e5
    valid_inflation = inflation_points[valid_mask]

    if len(valid_inflation) == 0:
        return 0.0

    # 计算每个 waypoint 到最近膨胀障碍物的欧式距离
    dists = np.linalg.norm(waypoints[:, None, :] - valid_inflation[None, :, :], axis=-1)
    min_dists = dists.min(axis=1)  # [H]

    # 惩罚参数
    collision_threshold = 0.05  # 0.05m 以内视为碰撞
    max_threshold = 0.2         # 0.2m 以上无惩罚
    per_point_penalty = 1.0 / predict_size

    # 初始化惩罚数组
    penalties = np.zeros(len(min_dists), dtype=np.float64)

    # 碰撞点：最大惩罚（不阻断后续点）
    in_collision = min_dists < collision_threshold
    penalties[in_collision] = per_point_penalty

    # 警告区域点：线性惩罚 [0.05m, 0.2m)
    in_warning = (min_dists >= collision_threshold) & (min_dists < max_threshold)
    penalties[in_warning] = (
        (max_threshold - min_dists[in_warning])
        / (max_threshold - collision_threshold)
        * per_point_penalty
    )

    return float(penalties.sum())


@register_reward_fn("occupancy")
class OccupancyRewardFn(RewardFn):
    """占用线性惩罚奖励函数。对每个waypoint到最近障碍物的欧式距离进行线性惩罚并累加。
    
    注意：将机器人位置视为质点，不考虑机器人半径。
    膨胀障碍物点云已经包含了安全距离的膨胀。
    """

    name: str = "occupancy"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.sampler = ObstacleSampler(
            sample_radius=float(self.config.get("sample_radius", 3.0)),
            obstacle_sample_n=int(self.config.get("obstacle_sample_n", 2048)),
        )

    def compute(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
        """计算 batch reward.

        Args:
            trajectories: rollout 轨迹列表, 每个元素必须包含 actions 和 inflation_points.
            **kwargs: device, dtype 等.

        Returns:
            torch.Tensor: 形状 [batch_size] 的 float reward, 值为累加线性惩罚 (0~1).
        
        Raises:
            ValueError: 如果轨迹缺少 inflation_points（必须由数据集预处理）
        """
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

        if len(trajectories) == 0:
            return torch.empty(0, device=device, dtype=dtype)

        try:
            actions = torch.stack([
                traj["actions"].to(device=device, dtype=dtype)
                for traj in trajectories
            ], dim=0)
            obstacle_tensors = []
            for traj_idx, traj in enumerate(trajectories):
                if "inflation_points" not in traj or traj["inflation_points"] is None:
                    raise ValueError(
                        f"轨迹 {traj_idx} 缺少 inflation_points！\n"
                        f"障碍物点云必须由数据集预处理（通过 relative_pose 转换到局部坐标系），\n"
                        f"以确保与轨迹的坐标系一致。\n"
                        f"请检查数据集是否正确加载了点云文件。"
                    )
                obstacle_tensors.append(traj["inflation_points"])

            first_obstacles = obstacle_tensors[0].to(device=device, dtype=dtype)
            same_obstacles = all(
                torch.is_tensor(obs)
                and obs.shape == first_obstacles.shape
                and obs.data_ptr() == obstacle_tensors[0].data_ptr()
                for obs in obstacle_tensors
            )

            if same_obstacles:
                if first_obstacles.shape[0] == 0:
                    return torch.zeros(len(trajectories), device=device, dtype=dtype)
                valid_mask = first_obstacles.abs().amax(dim=-1) < 1e5
                obstacles = first_obstacles[valid_mask, :2]
                if obstacles.shape[0] == 0:
                    return torch.zeros(len(trajectories), device=device, dtype=dtype)
                if all(traj.get("waypoints") is not None for traj in trajectories):
                    waypoints = torch.stack([
                        traj["waypoints"].to(device=device, dtype=dtype)
                        for traj in trajectories
                    ], dim=0)
                else:
                    waypoints = torch.cumsum(actions[:, :, :2] / 4.0, dim=1)  # [B, H, 2]
                dists = torch.cdist(waypoints, obstacles.unsqueeze(0).expand(waypoints.shape[0], -1, -1))
                min_dists = dists.amin(dim=-1)

                collision_threshold = 0.05
                max_threshold = 0.2
                # 与 compute_occupancy_reward_from_obstacles (fallback路径) 保持一致:
                # 分母使用实际预测步数, 两路径归一化口径相同.
                per_point_penalty = 1.0 / max(1, waypoints.shape[1])
                penalties = torch.zeros_like(min_dists)
                penalties = torch.where(
                    min_dists < collision_threshold,
                    torch.full_like(penalties, per_point_penalty),
                    penalties,
                )
                warning = (min_dists >= collision_threshold) & (min_dists < max_threshold)
                warning_penalty = (
                    (max_threshold - min_dists)
                    / (max_threshold - collision_threshold)
                    * per_point_penalty
                )
                penalties = torch.where(warning, warning_penalty, penalties)
                return penalties.sum(dim=1).to(dtype)
        except RuntimeError as e:
            # 仅允许因 tensor shape/device 不一致导致的 RuntimeError 回退到逐条路径.
            # ValueError (缺少必需字段) 不应被捕获，让其正常上抛。
            import warnings
            warnings.warn(
                f"OccupancyRewardFn 批量快速路径失败 ({e!r}), 回退到逐条计算. "
                f"如频繁出现请检查 actions/inflation_points 的 shape/device 一致性.",
                stacklevel=2,
            )
            pass

        rewards = []
        for traj_idx, traj in enumerate(trajectories):
            if traj.get("waypoints") is not None:
                waypoints = traj["waypoints"].to(device=device, dtype=dtype)
            else:
                actions = traj["actions"]
                waypoints = torch.from_numpy(
                    actions_to_waypoints(
                        actions,
                        action_scale=float(self.config.get("action_scale", 4.0)),
                    )
                ).to(device=device, dtype=dtype)

            # 只使用方式1: 预处理好的局部坐标膨胀障碍物
            # 不再支持世界坐标转换，确保坐标系一致性
            if "inflation_points" not in traj or traj["inflation_points"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 inflation_points！\n"
                    f"障碍物点云必须由数据集预处理（通过 relative_pose 转换到局部坐标系），\n"
                    f"以确保与轨迹的坐标系一致。\n"
                    f"请检查数据集是否正确加载了点云文件。"
                )

            inflation_obstacles = traj["inflation_points"].to(device)

            # 处理空障碍物数组：没有障碍物则无惩罚
            if inflation_obstacles.shape[0] == 0:
                rewards.append(0.0)
                continue

            # 基于膨胀障碍物点云计算占用比例
            waypoints_np = waypoints.cpu().numpy()
            inflation_np = inflation_obstacles.cpu().numpy()

            reward = compute_occupancy_reward_from_obstacles(
                waypoints_np,
                inflation_np,
                predict_size=waypoints_np.shape[0],
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)


def compute_projected_occupancy_reward(
    waypoints: np.ndarray,
    projected_obstacles: np.ndarray,
    predict_size: int = 24,
    collision_threshold: float = 0.05,
) -> float:
    """基于深度图投影障碍物点云计算占据奖励（无阻断逻辑）。

    与 compute_occupancy_reward_from_obstacles 的区别：
    - 不进行阻断处理（碰撞后后续点不再强制给最大惩罚）
    - 每个点独立计算线性惩罚

    惩罚规则：
    1. 若距离 < collision_threshold：惩罚 1/predict_size（不阻断后续点）
    2. 若距离在 [collision_threshold, 0.2m) 内：线性惩罚
    3. 若距离 >= 0.2m：惩罚为 0

    Args:
        waypoints: shape=[H, 2], 轨迹点序列 (x, y), 局部坐标系.
        projected_obstacles: shape=[N, 2], 深度图投影的障碍物点, 局部坐标系.
        predict_size: 轨迹预测步数，默认 24.
        collision_threshold: 碰撞阈值，默认 0.05m.

    Returns:
        float: 累加惩罚值, 范围 [0, 1].
    """
    if waypoints.size == 0:
        raise ValueError("waypoints 不能为空数组")

    if projected_obstacles.size == 0 or len(projected_obstacles) == 0:
        return 0.0

    # 过滤填充点 (1e6)
    valid_mask = np.abs(projected_obstacles).max(axis=-1) < 1e5
    valid_obs = projected_obstacles[valid_mask]

    if len(valid_obs) == 0:
        return 0.0

    # 计算每个 waypoint 到最近障碍物的欧式距离
    dists = np.linalg.norm(
        waypoints[:, None, :] - valid_obs[None, :, :], axis=-1
    )
    min_dists = dists.min(axis=1)

    # 惩罚参数
    max_threshold = 0.2  # 0.2m 以上无惩罚
    per_point_penalty = 1.0 / predict_size

    # 初始化惩罚数组
    penalties = np.zeros(len(min_dists), dtype=np.float64)

    # 碰撞点：最大惩罚（不阻断后续点）
    in_collision = min_dists < collision_threshold
    penalties[in_collision] = per_point_penalty

    # 警告区域点：线性惩罚
    in_warning = (min_dists >= collision_threshold) & (min_dists < max_threshold)
    penalties[in_warning] = (
        (max_threshold - min_dists[in_warning])
        / (max_threshold - collision_threshold)
        * per_point_penalty
    )

    return float(penalties.sum())


@register_reward_fn("projected_occupancy")
class ProjectedOccupancyRewardFn(RewardFn):
    """投影障碍物占据奖励函数。

    使用深度图投影的点作为障碍物，与 OccupancyRewardFn 的区别：
    - 不进行阻断处理（碰撞后后续点不再强制给最大惩罚）
    - 每个点独立计算线性惩罚

    坐标系转换链: depth → camera 3D → world 3D → (地面过滤) → local 2D
    与 test_depth_to_obstacle_reward.py 中的 depth_to_local_obstacles 完全一致。
    """

    name: str = "projected_occupancy"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        # 深度图单位：数据集返回的深度图已经是米单位（已除以 10000）
        # 所以 depth_scale 应该是 1.0，而不是 10000.0
        self.depth_scale = float(self.config.get("depth_scale", 1.0))
        self.min_depth = float(self.config.get("min_depth", 0.1))
        # 投影模式：不限制最大深度，使用一个很大的值
        self.max_depth = float(self.config.get("max_depth", 100.0))
        self.ground_threshold = float(self.config.get("ground_threshold", 0.4))
        self.collision_threshold = float(self.config.get("collision_threshold", 0.05))
        self.max_obstacle_points = int(self.config.get("max_obstacle_points", 4096))
        # v3 数据集标志：v3 的 relative_pose 包含 +90° 旋转，影响坐标转换
        self.is_v3_dataset = bool(self.config.get("is_v3_dataset", False))

    def _project_depth_to_local(
        self,
        depth_image: torch.Tensor,
        camera_intrinsic: torch.Tensor,
        camera_extrinsic: torch.Tensor,
        base_frame_extrinsic: torch.Tensor,
        base_extrinsic: torch.Tensor,
    ) -> torch.Tensor:
        """将深度图投影到局部坐标系。

        Args:
            depth_image: [H, W] or [H, W, 1] 深度图
            camera_intrinsic: [3, 3] 相机内参
            camera_extrinsic: [4, 4] 当前帧相机外参
            base_frame_extrinsic: [4, 4] 局部坐标系锚定帧外参
            base_extrinsic: [4, 4] 机器人基座外参

        Returns:
            [N, 2] 局部坐标系下的障碍物点
        """
        from .projected_collision_reward import (
            _create_relative_pose_numpy,
            depth_to_local_obstacles_numpy,
        )

        # 转换为 numpy
        depth_np = depth_image.cpu().numpy()
        # 处理可能的 channel 维度
        if depth_np.ndim == 3:
            depth_np = depth_np[:, :, 0]
        elif depth_np.ndim == 4:
            depth_np = depth_np[0, :, :, 0]

        intrinsic_np = camera_intrinsic.cpu().numpy()
        camera_ext_np = camera_extrinsic.cpu().numpy()
        base_frame_np = base_frame_extrinsic.cpu().numpy()
        base_ext_np = base_extrinsic.cpu().numpy()

        # 创建 numpy 版本的 relative_pose（v3 数据集需包含 +90° 旋转）
        relative_pose_np = _create_relative_pose_numpy(
            base_ext_np, v3_rotation=self.is_v3_dataset,
        )

        # 深度图投影
        # legacy 数据集: apply_x_flip=True（与 test_depth_to_obstacle_reward.py 一致）
        # v3 数据集: apply_x_flip=False（v3 的 relative_pose 已包含旋转，方向与轨迹一致）
        obstacles = depth_to_local_obstacles_numpy(
            depth_image=depth_np,
            camera_intrinsic=intrinsic_np,
            camera_extrinsic=camera_ext_np,
            base_frame_extrinsic=base_frame_np,
            base_extrinsic=base_ext_np,
            relative_pose_fn=relative_pose_np,
            depth_scale=self.depth_scale,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
            ground_threshold=self.ground_threshold,
            apply_x_flip=not self.is_v3_dataset,
        )

        if len(obstacles) == 0:
            return torch.zeros((0, 2), dtype=torch.float32)

        # 降采样以控制计算量
        if len(obstacles) > self.max_obstacle_points:
            rng = np.random.default_rng(42)
            indices = rng.choice(len(obstacles), self.max_obstacle_points, replace=False)
            obstacles = obstacles[indices]

        return torch.from_numpy(obstacles).float()

    def compute(
        self,
        trajectories: List[Dict[str, Any]],
        **kwargs: Any,
    ) -> torch.Tensor:
        """计算 batch reward.

        Args:
            trajectories: rollout 轨迹列表, 每个元素必须包含:
                - actions: [predict_size, 3] 增量动作
                - depth_image: [H, W] 深度图
                - camera_intrinsic: [3, 3] 相机内参
                - camera_extrinsic: [4, 4] 深度图同帧的相机外参
                - base_frame_extrinsic: [4, 4] 局部坐标系锚定帧外参
                - base_extrinsic: [4, 4] 机器人基座外参
            **kwargs: device, dtype 等.

        Returns:
            torch.Tensor: 形状 [batch_size] 的 float reward, 值为累加惩罚 (0~1).
        """
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

        # In GRPO every trajectory in a group shares the same observation, so
        # depth_image / intrinsics / extrinsics are the *same tensor objects*
        # across the batch. `_project_depth_to_local` is a pure function of those
        # inputs (its downsample uses a fixed rng seed), so projecting once per
        # unique input set and reusing the result is byte-identical to projecting
        # per trajectory — just ~B times cheaper. The memo is local to this call,
        # so a tensor address reused by a later call can never produce a stale hit.
        projection_memo: Dict[tuple, torch.Tensor] = {}

        def _project_cached(traj):
            key = (
                traj["depth_image"].data_ptr(),
                traj["camera_intrinsic"].data_ptr(),
                traj["camera_extrinsic"].data_ptr(),
                traj["base_frame_extrinsic"].data_ptr(),
                traj["base_extrinsic"].data_ptr(),
            )
            cached = projection_memo.get(key)
            if cached is not None:
                return cached
            obstacles = self._project_depth_to_local(
                depth_image=traj["depth_image"],
                camera_intrinsic=traj["camera_intrinsic"],
                camera_extrinsic=traj["camera_extrinsic"],
                base_frame_extrinsic=traj["base_frame_extrinsic"],
                base_extrinsic=traj["base_extrinsic"],
            )
            projection_memo[key] = obstacles
            return obstacles

        rewards = []
        for traj_idx, traj in enumerate(trajectories):
            if traj.get("waypoints") is not None:
                waypoints = traj["waypoints"].to(device=device, dtype=dtype)
            else:
                actions = traj["actions"]
                waypoints = torch.from_numpy(
                    actions_to_waypoints(
                        actions,
                        action_scale=float(self.config.get("action_scale", 4.0)),
                    )
                ).to(device=device, dtype=dtype)

            # 验证必需字段
            if "depth_image" not in traj or traj["depth_image"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 depth_image！\n"
                    f"投影障碍物占据奖励需要深度图进行实时投影。"
                )
            if "camera_intrinsic" not in traj or traj["camera_intrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 camera_intrinsic！\n"
                    f"投影障碍物占据奖励需要相机内参。"
                )
            if "camera_extrinsic" not in traj or traj["camera_extrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 camera_extrinsic！\n"
                    f"投影障碍物占据奖励需要当前帧相机外参。"
                )
            if "base_frame_extrinsic" not in traj or traj["base_frame_extrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 base_frame_extrinsic！\n"
                    f"投影障碍物占据奖励需要局部坐标系锚定帧外参。"
                )
            if "base_extrinsic" not in traj or traj["base_extrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 base_extrinsic！\n"
                    f"投影障碍物占据奖励需要机器人基座外参。"
                )

            # 深度图投影到局部坐标系（同一观测只投影一次，结果按 data_ptr 复用）
            projected_obstacles = _project_cached(traj)

            # 转换为 numpy 计算
            waypoints_np = waypoints.cpu().numpy()
            obstacles_np = projected_obstacles.cpu().numpy()

            reward = compute_projected_occupancy_reward(
                waypoints_np,
                obstacles_np,
                predict_size=waypoints_np.shape[0],
                collision_threshold=self.collision_threshold,
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)
