"""碰撞奖励 (CollisionRewardFn).

使用膨胀障碍物点云（蓝色点 [0, 0, 0.5]）进行碰撞检测。
与Occupancy奖励使用相同的障碍物点云。

碰撞检测逻辑：
    - 膨胀障碍物点云已包含膨胀半径（约0.30m），这是障碍物向外膨胀的距离
    - 机器人有实际半径（约0.25m）
    - 膨胀半径 > 机器人半径，差值约为 0.05m
    - 因此，当质点到膨胀障碍物表面距离 < collision_threshold 时视为碰撞
    - collision_threshold = 膨胀半径 - 机器人半径 ≈ 0.05m

输入: trajectories[i] 必须包含
    - ``actions``: [predict_size, 3] (dx, dy, dθ) 增量, 已 ×4 (NavDP 模型输出约定);
    - ``inflation_points``: [N, 2] 局部机器人坐标系下的膨胀障碍点 (xy) - 推荐，已转换好;
    - ``world_inflation_points``: [M, 2] or [M, 3] 世界坐标系下的膨胀障碍点;
    - ``start_world_pos``: [2] 轨迹起点在世界坐标系下的位置 (x, y) - 使用时必填;
    - ``base_frame_extrinsic``: [4, 4] 起点时刻的相机外参;
    - ``base_extrinsic``: [4, 4] 机器人基座外参.

输出: [batch_size] float tensor, 任一 waypoint 到最近膨胀障碍点距离
< collision_threshold 即视为碰撞, reward = ``collide_value`` (默认 1.0); 否则
``safe_value`` (默认 0.0).

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

import torch

from .base_reward import RewardFn, register_reward_fn
from .obstacle_sampler import ObstacleSampler
from .trajectory_utils import actions_to_waypoints


@register_reward_fn("collision")
class CollisionRewardFn(RewardFn):
    name: str = "collision"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        super().__init__(config)
        self.sampler = ObstacleSampler(
            sample_radius=float(self.config.get("sample_radius", 3.0)),
            obstacle_sample_n=int(self.config.get("obstacle_sample_n", 2048)),
        )
        # 碰撞阈值 = 膨胀半径 - 机器人半径 ≈ 0.30 - 0.25 = 0.05m
        # 当质点到膨胀障碍物距离 < 此阈值时视为碰撞
        self.collision_threshold = float(self.config.get("collision_threshold", 0.05))
        self.collide_value = float(self.config.get("collide_value", 1.0))
        self.safe_value = float(self.config.get("safe_value", 0.0))

    def compute(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
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
            
            obstacles = traj["inflation_points"].to(device)
            
            # 处理空障碍物数组：没有障碍物则不碰撞
            if obstacles.shape[0] == 0:
                rewards.append(self.safe_value)
                continue
            
            # 碰撞检测：使用 collision_threshold 判断
            # collision_threshold = 膨胀半径 - 机器人半径
            # 当质点到膨胀障碍物距离 < collision_threshold 时视为碰撞
            collided, _ = self.sampler.check_collision(
                waypoints, obstacles, robot_radius=self.collision_threshold
            )
            rewards.append(self.collide_value if collided else self.safe_value)
            
        return torch.tensor(rewards, device=device, dtype=dtype)
