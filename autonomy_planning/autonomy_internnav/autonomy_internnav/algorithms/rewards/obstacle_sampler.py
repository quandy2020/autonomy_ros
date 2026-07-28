"""障碍物采样工具模块

提供统一的障碍物采样功能，供所有 reward 函数共享使用。
避免重复计算，提高效率。

使用方式:
    from autonomy_internnav.algorithms.rewards.obstacle_sampler import ObstacleSampler
    
    sampler = ObstacleSampler(
        sample_radius=5.0,
        obstacle_sample_n=2048,
    )
    
    # 方式1: 使用预处理的局部坐标障碍物 (推荐)
    obstacles = traj.get("obstacle_points")  # 已转换到局部坐标系
    
    # 方式2: 世界坐标障碍物 + 外参
    local_obstacles = sampler.sample_and_transform(
        world_obstacles,
        base_frame_extrinsic,
        base_extrinsic,
    )
"""

from typing import Optional, Tuple
import torch


def world_to_local(
    world_points: torch.Tensor,
    base_frame_extrinsic: torch.Tensor,
    base_extrinsic: torch.Tensor,
) -> torch.Tensor:
    """将世界坐标系下的点转换到局部机器人坐标系
    
    坐标系转换逻辑与 navdp_lerobot_dataset.py 的 relative_pose 保持一致：
    - 单点: T_frame = [y, -x, z]
    - 多点: T_frame[:, [1, 0, 2]] 然后 T_frame[:, 1] = -T_frame[:, 1]
    
    Args:
        world_points: [N, 2] or [N, 3] 世界坐标系下的点
        base_frame_extrinsic: [4, 4] 起点时刻的相机外参
        base_extrinsic: [4, 4] 机器人基座外参
        
    Returns:
        [N, 2] 局部坐标系下的 xy 点
    """
    device = world_points.device
    dtype = world_points.dtype
    
    # 转换为 [N, 3] 格式
    if world_points.shape[-1] == 2:
        world_points_3d = torch.cat([
            world_points, 
            torch.zeros(len(world_points), 1, device=device, dtype=dtype)
        ], dim=-1)
    else:
        world_points_3d = world_points
    
    # 相对位姿计算 (参考 navdp_lerobot_dataset.py)
    R_base = base_frame_extrinsic[:3, :3] @ torch.inverse(base_extrinsic[:3, :3])
    T_base = base_frame_extrinsic[:3, 3]
    
    homo_RT = torch.eye(4, device=device, dtype=dtype)
    homo_RT[:3, :3] = R_base
    homo_RT[:3, 3] = T_base
    
    # 世界坐标转局部坐标
    world_homo = torch.cat([world_points_3d, torch.ones(len(world_points_3d), 1, device=device, dtype=dtype)], dim=-1)
    local = torch.linalg.inv(homo_RT) @ world_homo.T
    local = local[:3].T
    
    # 坐标变换 [x, y, z] -> [y, -x, z] (与 navdp_lerobot_dataset.py 的 relative_pose 一致)
    # T_frame = T_frame[:, [1, 0, 2]]
    # T_frame[:, 1] = -T_frame[:, 1]
    local_xy = torch.stack([local[:, 1], -local[:, 0]], dim=-1)
    
    return local_xy


class ObstacleSampler:
    """障碍物采样器
    
    统一管理障碍物的采样和坐标系转换，供所有 reward 函数共享使用。
    
    不进行降采样，只保留sample_radius范围内的点。
    
    Attributes:
        sample_radius: 采样半径 (米)
        obstacle_sample_n: 已弃用，保留用于兼容性
    """
    
    def __init__(
        self,
        sample_radius: float = 3.0,
        obstacle_sample_n: int = 2048,  # 保留参数用于兼容性，但不再使用
    ):
        self.sample_radius = sample_radius
        self.obstacle_sample_n = obstacle_sample_n
    
    def sample_and_transform(
        self,
        world_obstacles: torch.Tensor,
        base_frame_extrinsic: Optional[torch.Tensor] = None,
        base_extrinsic: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """转换障碍物点到局部坐标系，只保留sample_radius范围内的点
        
        不进行降采样，保留所有在sample_radius范围内的点。
        
        Args:
            world_obstacles: [M, 2] or [M, 3] 世界坐标系下的障碍物点
            base_frame_extrinsic: [4, 4] 起点时刻的相机外参 (可选)
            base_extrinsic: [4, 4] 机器人基座外参 (可选)
            
        Returns:
            [N, 2] 局部坐标系下的障碍物点，N为sample_radius范围内的点数
        """
        device = world_obstacles.device
        dtype = world_obstacles.dtype
        
        # 只取 xy 坐标
        if world_obstacles.shape[-1] == 3:
            world_obstacles_xy = world_obstacles[..., :2]
        else:
            world_obstacles_xy = world_obstacles
        
        # 过滤填充点 (1e6)
        valid_mask = (world_obstacles_xy.abs() < 1e5).all(dim=-1)
        valid_obstacles = world_obstacles_xy[valid_mask]
        
        if len(valid_obstacles) == 0:
            return torch.zeros((0, 2), device=device, dtype=dtype)
        
        # 转换到局部坐标系
        if base_frame_extrinsic is not None and base_extrinsic is not None:
            # 需要 3D 点进行坐标转换
            if world_obstacles.shape[-1] == 2:
                world_3d = torch.cat([
                    world_obstacles,
                    torch.zeros(len(world_obstacles), 1, device=device, dtype=dtype)
                ], dim=-1)
            else:
                world_3d = world_obstacles
            
            # 只对有效点进行转换
            valid_world_3d = world_3d[valid_mask]
            
            local_3d = world_to_local(
                valid_world_3d, 
                base_frame_extrinsic, 
                base_extrinsic
            )
            # 转换后只取 xy
            local_obstacles = local_3d[..., :2]
        else:
            # 假设起点在原点，只使用 xy
            local_obstacles = valid_obstacles
        
        # 计算起点到各障碍物的距离 (欧式距离)
        origin = torch.zeros(2, device=device, dtype=dtype)
        distances = torch.norm(local_obstacles - origin, dim=-1)
        
        # 只保留半径范围内的障碍物，不进行随机降采样
        in_range_mask = distances <= self.sample_radius
        in_range_obstacles = local_obstacles[in_range_mask]
        
        return in_range_obstacles
    
    def compute_euclidean_distances(
        self,
        waypoints: torch.Tensor,
        obstacles: torch.Tensor,
    ) -> torch.Tensor:
        """计算每个 waypoint 到最近障碍物的欧式距离
        
        Args:
            waypoints: [H, 2] or [H, 3] 轨迹点
            obstacles: [N, 2] or [N, 3] 障碍物点
            
        Returns:
            [H] 每个 waypoint 到最近障碍物的欧式距离
        """
        # 只取 xy 坐标
        waypoints_xy = waypoints[..., :2] if waypoints.shape[-1] >= 2 else waypoints
        obstacles_xy = obstacles[..., :2] if obstacles.shape[-1] >= 2 else obstacles
        
        # 处理空障碍物数组
        if obstacles_xy.shape[0] == 0:
            # 返回无穷大距离，表示没有障碍物
            return torch.full((waypoints_xy.shape[0],), float('inf'), device=waypoints_xy.device, dtype=waypoints_xy.dtype)
        
        dists = torch.norm(waypoints_xy[:, None, :] - obstacles_xy[None, :, :], dim=-1)
        return dists.min(dim=-1).values
    
    def check_collision(
        self,
        waypoints: torch.Tensor,
        obstacles: torch.Tensor,
        robot_radius: float = 0.2,
    ) -> Tuple[bool, torch.Tensor]:
        """检测轨迹是否与障碍物碰撞
        
        任一点 waypoint 到最近障碍物的欧式距离 < robot_radius 即视为碰撞。
        
        Args:
            waypoints: [H, 2] or [H, 3] 轨迹点 (局部坐标系)
            obstacles: [N, 2] or [N, 3] 障碍物点 (局部坐标系)
            robot_radius: 机器人半径
            
        Returns:
            (是否碰撞, 每个 waypoint 到最近障碍物的距离)
        """
        # 处理空障碍物数组
        if obstacles.shape[0] == 0:
            # 没有障碍物，不碰撞
            return False, torch.full((waypoints.shape[0],), float('inf'), device=waypoints.device, dtype=waypoints.dtype)
        
        min_distances = self.compute_euclidean_distances(waypoints, obstacles)
        collided = bool((min_distances < robot_radius).any().item())
        return collided, min_distances
    
    def transform_world_obstacles_to_local(
        self,
        world_obstacles: torch.Tensor,
        start_world_pos: torch.Tensor,
    ) -> torch.Tensor:
        """将世界坐标障碍物转换到局部坐标系 (以起点为原点)
        
        假设:
        - waypoints 是累积值，从 (0, 0) 开始
        - 障碍物在世界坐标系下
        - 起点在世界坐标系下的位置为 start_world_pos
        
        Args:
            world_obstacles: [M, 2] 世界坐标系下的障碍物点
            start_world_pos: [2] 轨迹起点在世界坐标系下的位置 (x, y)
            
        Returns:
            [M, 2] 转换到局部坐标系下的障碍物点
        """
        # 只取 xy 坐标
        if world_obstacles.shape[-1] >= 2:
            world_xy = world_obstacles[..., :2]
        else:
            world_xy = world_obstacles
        
        # 转换: local = world - start_pos
        start_pos = start_world_pos[..., :2] if start_world_pos.shape[-1] >= 2 else start_world_pos
        local_obstacles = world_xy - start_pos
        
        return local_obstacles


# 全局默认采样器
_default_sampler = None


def get_default_sampler() -> ObstacleSampler:
    """获取全局默认采样器"""
    global _default_sampler
    if _default_sampler is None:
        _default_sampler = ObstacleSampler()
    return _default_sampler


def filter_obstacles_by_fov(
    local_obstacles: torch.Tensor,
    fov_deg: float = 180.0,
) -> torch.Tensor:
    """过滤局部坐标系下的障碍物，只保留前方FOV范围内的点
    
    在局部机器人坐标系中：
    - 机器人位于原点 (0, 0)
    - 前方方向为 x 轴正方向
    - 左侧为 y 轴正方向
    - 角度计算: angle = atan2(y, x)
    
    FOV定义：
    - fov_deg=180 表示前方180度（左侧90度到右侧90度）
    - 保留条件: |angle| <= fov_deg / 2
    
    Args:
        local_obstacles: [N, 2] 局部坐标系下的障碍物点 (x, y)
        fov_deg: 视场角（度），默认180度（前方左右各90度）
        
    Returns:
        [M, 2] 过滤后的障碍物点，M <= N
    """
    if local_obstacles.shape[0] == 0:
        return local_obstacles
    
    device = local_obstacles.device
    dtype = local_obstacles.dtype
    
    # 计算每个障碍物点相对于机器人的角度
    # angle = atan2(y, x)，范围 [-π, π]
    # x轴正方向为前方（角度=0）
    # y轴正方向为左侧（角度=+π/2）
    angles = torch.atan2(local_obstacles[:, 1], local_obstacles[:, 0])
    
    # 计算角度阈值（弧度）
    fov_rad = torch.deg2rad(torch.tensor(fov_deg, device=device, dtype=dtype))
    half_fov = fov_rad / 2.0
    
    # 保留前方FOV范围内的点：|angle| <= half_fov
    in_fov_mask = torch.abs(angles) <= half_fov
    filtered_obstacles = local_obstacles[in_fov_mask]
    
    return filtered_obstacles


def sample_obstacles(
    world_obstacles: torch.Tensor,
    base_frame_extrinsic: Optional[torch.Tensor] = None,
    base_extrinsic: Optional[torch.Tensor] = None,
    sample_radius: float = 5.0,
    obstacle_sample_n: int = 2048,
) -> torch.Tensor:
    """快捷函数：采样障碍物点
    
    使用全局默认采样器进行采样。
    """
    sampler = get_default_sampler()
    sampler.sample_radius = sample_radius
    sampler.obstacle_sample_n = obstacle_sample_n
    
    return sampler.sample_and_transform(
        world_obstacles,
        base_frame_extrinsic,
        base_extrinsic,
    )