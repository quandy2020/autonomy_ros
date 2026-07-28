"""投影障碍物碰撞奖励 (ProjectedCollisionRewardFn).

使用深度图投影的点作为障碍物进行碰撞检测。
- 每个点单独进行碰撞检测
- 如果碰撞，每个点进行惩罚扣分
- 每条轨迹的分数由点进行累计

坐标系转换链: depth → camera 3D → world 3D → (地面过滤) → local 2D
与 test_depth_to_obstacle_reward.py 中的 depth_to_local_obstacles 完全一致。

输入: trajectories[i] 必须包含
    - ``actions``: [predict_size, 3] (dx, dy, dθ) 增量, 已 ×4;
    - ``depth_image``: [H, W] 深度图 (uint16)
    - ``camera_intrinsic``: [3, 3] 相机内参
    - ``camera_extrinsic``: [4, 4] 当前帧相机外参
    - ``base_frame_extrinsic``: [4, 4] 局部坐标系锚定帧外参
    - ``base_extrinsic``: [4, 4] 机器人基座外参
"""

from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import torch

from .base_reward import RewardFn, register_reward_fn
from .trajectory_utils import actions_to_waypoints


def depth_to_pointcloud(
    depth_image: np.ndarray,
    camera_intrinsic: np.ndarray,
    depth_scale: float = 10000.0,
    min_depth: float = 0.1,
    max_depth: float = 5.0,
) -> np.ndarray:
    """深度图反投影为相机坐标系 3D 点云.

    相机坐标系约定: x=右, y=下, z=前(深度/光轴方向).
    本函数只做标准相机坐标系反投影, 不做俯仰角修正.
    后续 pointcloud_camera_to_world 会用完整的 camera-to-world 外参矩阵
    (已包含俯仰角旋转) 将点云变换到世界坐标系.

    Args:
        depth_image: [H, W] uint16 深度图.
        camera_intrinsic: [3, 3] 相机内参.
        depth_scale: pixel_value / depth_scale = 深度(米).
        min_depth: 最小有效深度 (米).
        max_depth: 最大有效深度 (米).

    Returns:
        points_camera: [N, 3] 相机坐标系点 (x右, y下, z前).
    """
    H, W = depth_image.shape
    fx, fy = camera_intrinsic[0, 0], camera_intrinsic[1, 1]
    cx, cy = camera_intrinsic[0, 2], camera_intrinsic[1, 2]

    u_coords, v_coords = np.meshgrid(np.arange(W), np.arange(H))
    depth_m = depth_image.astype(np.float64) / depth_scale

    valid_mask = (depth_m > min_depth) & (depth_m < max_depth)
    u_valid = u_coords[valid_mask].astype(np.float64)
    v_valid = v_coords[valid_mask].astype(np.float64)
    d_valid = depth_m[valid_mask]

    # 标准相机坐标系反投影 (光轴沿 z, x=右, y=下)
    x = (u_valid - cx) * d_valid / fx
    y = (v_valid - cy) * d_valid / fy
    z = d_valid
    points_camera = np.stack([x, y, z], axis=-1)  # [N, 3]

    return points_camera


def pointcloud_camera_to_world(
    points_camera: np.ndarray,
    camera_extrinsic: np.ndarray,
) -> np.ndarray:
    """相机坐标系 → 世界坐标系.

    Args:
        points_camera: [N, 3]
        camera_extrinsic: [4, 4] 第 i 帧的外参 (trajectory_extrinsics[i])

    Returns:
        points_world: [N, 3]
    """
    N = points_camera.shape[0]
    ones = np.ones((N, 1), dtype=points_camera.dtype)
    points_homo = np.concatenate([points_camera, ones], axis=-1)
    points_world_homo = (camera_extrinsic @ points_homo.T).T
    return points_world_homo[:, :3]


def depth_to_local_obstacles_numpy(
    depth_image: np.ndarray,
    camera_intrinsic: np.ndarray,
    camera_extrinsic: np.ndarray,
    base_frame_extrinsic: np.ndarray,
    base_extrinsic: np.ndarray,
    relative_pose_fn: Callable,
    depth_scale: float = 10000.0,
    min_depth: float = 0.1,
    max_depth: float = 5.0,
    ground_threshold: float = 0.4,
    apply_x_flip: bool = True,
) -> np.ndarray:
    """从深度图提取 2D BEV 障碍物点, 统一在机器人局部坐标系下.

    坐标系转换链: depth → camera 3D → world 3D → (地面过滤) → local 2D

    与数据集 _obstacles_to_local 以及 test_depth_to_obstacle_reward.py 中的
    depth_to_local_obstacles 完全一致:
      调用 relative_pose(R_base=base_frame_extrinsic[:3,:3],
                         T_base=base_frame_extrinsic[:3,3],
                         R_world=I, T_world=world_points,
                         base_extrinsic)

    relative_pose 输出约定 (NavDP_Base_Datset): local = [inv_y, -inv_x, inv_z]
      local_x = inv_y = 前进方向, local_y = -inv_x = 侧向

    地面过滤 (在世界坐标系下, z 轴为高度方向, z 朝下):
      z 越大 = 越低 (地面方向); z 越小 = 越高 (天花板方向).
      地面参考 = 点云 z 的 95th 百分位 (z 最大的点即地面).
      保留离地面超过 ground_threshold 高度的障碍物:
        world_z < (ground_z - ground_threshold)

    Args:
        depth_image: [H, W] uint16 深度图.
        camera_intrinsic: [3, 3] 相机内参.
        camera_extrinsic: [4, 4] 深度图同帧的相机外参.
        base_frame_extrinsic: [4, 4] 局部坐标系锚定帧 (GT第0帧外参).
        base_extrinsic: [4, 4] 机器人基座外参.
        relative_pose_fn: 数据集的 relative_pose 方法.
        depth_scale: pixel_value / depth_scale = 深度(米).
        min_depth: 最小有效深度 (米).
        max_depth: 最大有效深度 (米).
        ground_threshold: 离地面高度阈值 (米), 保留离地面 > 此高度的障碍物点.
        apply_x_flip: 是否对 x 轴取反 (y轴镜像)。
            legacy 数据集 (NavDP_Base_Datset): 需要 x 取反使障碍物与轨迹对齐
            v3 数据集 (NavDP_LerobotV3_Dataset): 不需要 x 取反
            （v3 的 relative_pose 已包含 +90° 旋转，输出方向与轨迹一致）

    Returns:
        obstacle_local_xy: [M, 2] 局部坐标系障碍物点 (与轨迹同一坐标系).
    """
    # 1. depth → camera 3D (标准反投影, 不做俯仰角修正)
    points_camera = depth_to_pointcloud(
        depth_image, camera_intrinsic, depth_scale, min_depth, max_depth,
    )

    if points_camera.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float32)

    # 2. camera 3D → world 3D (全部点, 先转换再过滤)
    points_world = pointcloud_camera_to_world(points_camera, camera_extrinsic)

    # 3. 地面过滤 (在世界坐标系下, z 轴为高度方向, z 朝下)
    #    z 越大 = 越低 (地面方向); z 越小 = 越高 (天花板方向)
    #    相机安装在机器人上, 相机 z 坐标 = camera_extrinsic[2,3]
    #    地面在相机下方 (z 更大), 天花板在相机上方 (z 更小)
    #    从地面往上数 ground_threshold 高度内的点都去除:
    #    地面参考 = 点云中 z 最大处 (95th 百分位估计, 避免噪声)
    #    保留 z < ground_z - ground_threshold 的点 (即离地面超过 threshold 的障碍物)
    ground_z_estimate = np.percentile(points_world[:, 2], 95)  # 地面: z 最大的点
    height_mask = points_world[:, 2] < (ground_z_estimate - ground_threshold)
    points_above_ground_world = points_world[height_mask]

    if points_above_ground_world.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float32)

    # 4. world 3D → local 2D (调用 relative_pose)
    _, local = relative_pose_fn(
        base_frame_extrinsic[0:3, 0:3],
        base_frame_extrinsic[0:3, 3],
        np.eye(3),
        points_above_ground_world,
        base_extrinsic,
    )
    obstacle_local_xy = local[:, 0:2].astype(np.float32)

    # x 轴取反 (y轴镜像)
    # legacy 数据集: 需要 x 取反使障碍物与轨迹对齐
    #   (与 test_depth_to_obstacle_reward.py 中的 depth_to_local_obstacles 一致)
    # v3 数据集: 不需要 x 取反 (relative_pose 已包含 +90° 旋转，方向与轨迹一致)
    if apply_x_flip:
        obstacle_local_xy[:, 0] = -obstacle_local_xy[:, 0]  # x 取反 (y轴镜像)

    return obstacle_local_xy


def compute_projected_collision_reward(
    waypoints: np.ndarray,
    projected_obstacles: np.ndarray,
    robot_radius: float = 0.2,
) -> float:
    """计算投影障碍物碰撞奖励.

    每个点单独进行碰撞检测，碰撞点进行惩罚扣分，累加所有碰撞点的惩罚。

    Args:
        waypoints: shape=[H, 2], 轨迹点序列 (x, y), 局部坐标系.
        projected_obstacles: shape=[N, 2], 深度图投影的障碍物点, 局部坐标系.
        robot_radius: 机器人半径.

    Returns:
        float: 累加惩罚值，每个碰撞点惩罚 = 1/num_waypoints.
    """
    if waypoints.size == 0:
        return 0.0

    if projected_obstacles.size == 0 or len(projected_obstacles) == 0:
        return 0.0

    # 过滤填充点 (1e6)
    valid_mask = np.abs(projected_obstacles).max(axis=-1) < 1e5
    valid_obs = projected_obstacles[valid_mask]

    if len(valid_obs) == 0:
        return 0.0

    num_waypoints = len(waypoints)
    per_point_penalty = 1.0 / num_waypoints

    # 计算每个 waypoint 到最近障碍物的距离
    dists = np.linalg.norm(
        waypoints[:, None, :] - valid_obs[None, :, :], axis=-1
    )
    min_dists = dists.min(axis=1)

    # 碰撞检测：距离 < robot_radius 视为碰撞
    collision_mask = min_dists < robot_radius
    num_collisions = collision_mask.sum()

    return float(num_collisions * per_point_penalty)


def _create_relative_pose_numpy(base_extrinsic: np.ndarray, v3_rotation: bool = False):
    """创建与数据集 relative_pose 等价的 numpy 函数.

    Args:
        base_extrinsic: [4, 4] 机器人基座外参
        v3_rotation: 是否应用 v3 数据集的 +90° 旋转。
            NavDP_LerobotV3_Dataset.relative_pose 在父类基础上额外做了 +90° 旋转
            (x' = -y, y' = x)，使 v3 的 y-forward 坐标系与 NavDP 的 x-forward 对齐。
            当使用 v3 数据集时必须设为 True，否则投影障碍物与轨迹坐标系不一致。

    Returns:
        relative_pose_fn: 与数据集 relative_pose 方法签名一致的函数
    """
    def relative_pose(
        R_base: np.ndarray,
        T_base: np.ndarray,
        R_world: np.ndarray,
        T_world: np.ndarray,
        base_extrinsic: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """numpy 版本的 relative_pose.

        与 NavDP_Base_Datset.relative_pose 完全一致:
        - 单点: T_frame = [y, -x, z]
        - 多点: T_frame[:, [1, 0, 2]] 然后 T_frame[:, 1] = -T_frame[:, 1]

        当 v3_rotation=True 时，额外做 +90° 旋转:
        - x' = -y, y' = x
        与 NavDP_LerobotV3_Dataset.relative_pose 的覆写行为一致。
        """
        R_base_new = np.matmul(R_base, np.linalg.inv(base_extrinsic[0:3, 0:3]))

        if len(T_world.shape) == 1:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base_new
            homo_RT[0:3, 3] = T_base
            R_frame = np.dot(R_world, R_base_new.T)
            T_frame = np.dot(np.linalg.inv(homo_RT), np.array([*T_world, 1]).T)[0:3]
            T_frame = np.array([T_frame[1], -T_frame[0], T_frame[2]])
            if v3_rotation:
                x_old, y_old = T_frame[0], T_frame[1]
                T_frame[0] = -y_old
                T_frame[1] = x_old
            return R_frame, T_frame
        else:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base_new
            homo_RT[0:3, 3] = T_base
            R_frame = np.dot(R_world, R_base_new.T)
            T_frame = np.dot(
                np.linalg.inv(homo_RT),
                np.concatenate((T_world, np.ones((T_world.shape[0], 1))), axis=-1).T
            ).T[:, 0:3]
            T_frame = T_frame[:, [1, 0, 2]]
            T_frame[:, 1] = -T_frame[:, 1]
            if v3_rotation:
                x_old = T_frame[:, 0].copy()
                y_old = T_frame[:, 1].copy()
                T_frame[:, 0] = -y_old
                T_frame[:, 1] = x_old
            return R_frame, T_frame

    return relative_pose


@register_reward_fn("projected_collision")
class ProjectedCollisionRewardFn(RewardFn):
    """投影障碍物碰撞奖励函数。

    使用深度图投影的点作为障碍物，每个点单独碰撞检测并累加惩罚。
    与 OccupancyRewardFn 和 CollisionRewardFn 的区别：
    - 障碍物来源不同：使用当前帧深度图实时投影，而非预处理的点云文件
    - 每个点独立检测，无阻断逻辑
    """

    name: str = "projected_collision"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.robot_radius = float(self.config.get("robot_radius", 0.2))
        # 深度图单位：数据集返回的深度图已经是米单位（已除以 10000）
        # 所以 depth_scale 应该是 1.0，而不是 10000.0
        self.depth_scale = float(self.config.get("depth_scale", 1.0))
        self.min_depth = float(self.config.get("min_depth", 0.1))
        # 投影模式：不限制最大深度，使用一个很大的值
        self.max_depth = float(self.config.get("max_depth", 100.0))
        self.ground_threshold = float(self.config.get("ground_threshold", 0.4))
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
            torch.Tensor: 形状 [batch_size] 的 float reward, 值为累加碰撞惩罚 (0~1).
        """
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

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
                        start_xy=traj.get("start_pos"),
                    )
                ).to(device=device, dtype=dtype)

            # 验证必需字段
            if "depth_image" not in traj or traj["depth_image"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 depth_image！\n"
                    f"投影障碍物奖励需要深度图进行实时投影。"
                )
            if "camera_intrinsic" not in traj or traj["camera_intrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 camera_intrinsic！\n"
                    f"投影障碍物奖励需要相机内参。"
                )
            if "camera_extrinsic" not in traj or traj["camera_extrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 camera_extrinsic！\n"
                    f"投影障碍物奖励需要当前帧相机外参。"
                )
            if "base_frame_extrinsic" not in traj or traj["base_frame_extrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 base_frame_extrinsic！\n"
                    f"投影障碍物奖励需要局部坐标系锚定帧外参。"
                )
            if "base_extrinsic" not in traj or traj["base_extrinsic"] is None:
                raise ValueError(
                    f"轨迹 {traj_idx} 缺少 base_extrinsic！\n"
                    f"投影障碍物奖励需要机器人基座外参。"
                )

            # 深度图投影到局部坐标系
            projected_obstacles = self._project_depth_to_local(
                depth_image=traj["depth_image"],
                camera_intrinsic=traj["camera_intrinsic"],
                camera_extrinsic=traj["camera_extrinsic"],
                base_frame_extrinsic=traj["base_frame_extrinsic"],
                base_extrinsic=traj["base_extrinsic"],
            )

            # 转换为 numpy 计算
            waypoints_np = waypoints.cpu().numpy()
            obstacles_np = projected_obstacles.cpu().numpy()

            reward = compute_projected_collision_reward(
                waypoints_np,
                obstacles_np,
                robot_radius=self.robot_radius,
            )
            rewards.append(reward)

        return torch.tensor(rewards, device=device, dtype=dtype)
