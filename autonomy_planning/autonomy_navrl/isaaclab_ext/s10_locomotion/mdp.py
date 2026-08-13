"""S10-specific MDP helpers (wheel-aware joint observations)."""

from __future__ import annotations

import torch
from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import SceneEntityCfg

# Must match S10_WHEEL_JOINTS order in s10_velocity_env_cfg / s10_jit.
_S10_WHEEL_JOINT_NAMES = (
    'fl_wheel_joint',
    'fr_wheel_joint',
    'hl_wheel_joint',
    'hr_wheel_joint',
)


def joint_pos_rel_without_wheel(
    env: ManagerBasedEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg('robot'),
) -> torch.Tensor:
    """Relative joint positions with continuous wheel slots forced to 0.

    Matches Go2W ``joint_pos_rel_without_wheel`` so wheel angle drift does not
    pollute the policy observation.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    joint_ids = asset_cfg.joint_ids
    pos_rel = asset.data.joint_pos[:, joint_ids] - asset.data.default_joint_pos[:, joint_ids]

    names = list(asset.data.joint_names)
    if joint_ids == slice(None):
        selected = names
    else:
        selected = [names[i] for i in joint_ids]
    for wheel_name in _S10_WHEEL_JOINT_NAMES:
        if wheel_name in selected:
            pos_rel[:, selected.index(wheel_name)] = 0.0
    return pos_rel
