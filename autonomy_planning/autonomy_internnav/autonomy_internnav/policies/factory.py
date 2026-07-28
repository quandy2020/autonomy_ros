"""Policy factory and capability registry."""

from __future__ import annotations

from autonomy_internnav.config import Config
from autonomy_internnav.policies.base import PolicyInference
from autonomy_internnav.policies.internvla_n1_policy import InternVLAN1PolicyInference
from autonomy_internnav.policies.logoplanner_policy import LoGoPlannerPolicyInference
from autonomy_internnav.policies.navdp_policy import NavDPPolicyInference
from autonomy_internnav.policies.nomad_policy import NoMaDPolicyInference
from autonomy_internnav.policies.vint_policy import ViNTPolicyInference
from autonomy_internnav.policies.viplanner_policy import VIPlannerPolicyInference

POLICY_CLASSES: dict[str, type[PolicyInference]] = {
    'navdp': NavDPPolicyInference,
    'internvla_n1': InternVLAN1PolicyInference,
    'logoplanner': LoGoPlannerPolicyInference,
    'viplanner': VIPlannerPolicyInference,
    'vint': ViNTPolicyInference,
    'nomad': NoMaDPolicyInference,
}

POINT_GOAL_POLICIES = frozenset({'navdp', 'internvla_n1', 'logoplanner', 'viplanner'})
IMAGE_GOAL_POLICIES = frozenset({'navdp', 'internvla_n1', 'vint', 'nomad'})
NOGOAL_POLICIES = frozenset({'navdp', 'internvla_n1', 'vint', 'nomad'})


def create_policy_inference(cfg: Config, intrinsic=None) -> PolicyInference:
    """Instantiate the selected navigation policy."""
    policy = cfg.policy.lower().strip()
    if policy not in POLICY_CLASSES:
        supported = ', '.join(sorted(POLICY_CLASSES))
        raise ValueError(f'Unknown policy {cfg.policy!r}. Supported: {supported}')

    goal_type = cfg.goal_type.lower().strip()
    if goal_type == 'point' and policy not in POINT_GOAL_POLICIES:
        raise ValueError(
            f'Policy {policy!r} does not support goal_type=point. '
            f'Use one of: {sorted(POINT_GOAL_POLICIES)} or set goal_type:=image')
    if goal_type == 'image' and policy not in IMAGE_GOAL_POLICIES:
        raise ValueError(
            f'Policy {policy!r} does not support goal_type=image. '
            f'Use one of: {sorted(IMAGE_GOAL_POLICIES)}')
    if goal_type == 'nogoal' and policy not in NOGOAL_POLICIES:
        raise ValueError(
            f'Policy {policy!r} does not support goal_type=nogoal. '
            f'Use one of: {sorted(NOGOAL_POLICIES)}')

    return POLICY_CLASSES[policy](cfg, intrinsic)
