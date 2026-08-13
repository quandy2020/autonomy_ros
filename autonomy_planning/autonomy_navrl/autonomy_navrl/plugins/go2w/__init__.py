"""Go2W-specific plugins and utilities."""

from autonomy_navrl.plugins.go2w.defaults import (
    GO2W_INIT_JOINT_POS,
    GO2W_ISAAC_DEFAULTS,
    GO2W_PRECISION_REWARD,
    GO2W_PRECISION_TASK,
    GO2W_URDF_REL,
)
from autonomy_navrl.plugins.go2w.iou import (
    GO2W_ROBOT_LENGTH,
    GO2W_ROBOT_WIDTH,
    GO2W_SPOT_LENGTH,
    GO2W_SPOT_WIDTH,
    bbox_iou_from_body_pose,
)
from autonomy_navrl.plugins.go2w.success_tracker import ConsecutiveIoUConfig, ConsecutiveIoUTracker

__all__ = [
    'GO2W_INIT_JOINT_POS',
    'GO2W_ISAAC_DEFAULTS',
    'GO2W_PRECISION_REWARD',
    'GO2W_PRECISION_TASK',
    'GO2W_ROBOT_LENGTH',
    'GO2W_ROBOT_WIDTH',
    'GO2W_SPOT_LENGTH',
    'GO2W_SPOT_WIDTH',
    'GO2W_URDF_REL',
    'ConsecutiveIoUConfig',
    'ConsecutiveIoUTracker',
    'bbox_iou_from_body_pose',
]
