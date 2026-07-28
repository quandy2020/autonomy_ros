"""奖励函数子模块. 当前仅提供抽象接口与 dummy 实现, 后续按任务扩展."""

from .base_reward import DummyRewardFn, RewardFn, build_reward_fn
from .collision_reward import CollisionRewardFn  # noqa: F401  (触发注册)
from .composite_reward import CompositeRewardFn  # noqa: F401  (触发注册)
from .occupancy_reward import OccupancyRewardFn, ProjectedOccupancyRewardFn  # noqa: F401  (触发注册)
from .progress_reward import ProgressRewardFn  # noqa: F401  (触发注册)
from .projected_collision_reward import ProjectedCollisionRewardFn  # noqa: F401  (触发注册)
from .smooth_reward import SmoothRewardFn  # noqa: F401  (触发注册)
from .speed_limit_reward import SpeedLimitRewardFn  # noqa: F401  (触发注册, legacy 兼容)
from .success_reward import SuccessRewardFn  # noqa: F401  (触发注册)
from .target_speed_reward import TargetSpeedRewardFn  # noqa: F401  (触发注册)
from .terminal_heading_reward import TerminalHeadingRewardFn  # noqa: F401  (触发注册)
from .trajectory_utils import actions_to_waypoints, actions_to_waypoints_with_start  # noqa: F401
from .goal_direction_projection_reward import GoalDirectionProjectionRewardFn  # noqa: F401  (触发注册)
from .ttc_reward import TTCRewardFn  # noqa: F401  (触发注册)

__all__ = [
    "RewardFn",
    "DummyRewardFn",
    "CompositeRewardFn",
    "CollisionRewardFn",
    "OccupancyRewardFn",
    "ProjectedCollisionRewardFn",
    "ProjectedOccupancyRewardFn",
    "ProgressRewardFn",
    "SpeedLimitRewardFn",
    "SuccessRewardFn",
    "TargetSpeedRewardFn",
    "SmoothRewardFn",
    "TerminalHeadingRewardFn",
    "TTCRewardFn",
    "GoalDirectionProjectionRewardFn",
    "build_reward_fn",
]