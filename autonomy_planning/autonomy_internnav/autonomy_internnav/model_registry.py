"""Policy registry aligned with JdInternNav internnav.model."""

from __future__ import annotations


def get_policy(policy_name: str):
    if policy_name in ('NavDP_Policy', 'navdp_Policy'):
        from autonomy_internnav.train.navdp_model import NavDPNet

        return NavDPNet
    if policy_name in ('NavDPGRPO_Policy', 'navdp_grpo_Policy'):
        from autonomy_internnav.train.navdp_grpo_model import NavDPGRPONet

        return NavDPGRPONet
    if policy_name == 'InternVLAN1_Policy':
        from autonomy_internnav.internvla_n1.model.internvla_n1_policy import InternVLAN1Net

        return InternVLAN1Net
    raise ValueError(f'Policy {policy_name!r} not found')


def get_config(policy_name: str):
    if policy_name in ('NavDP_Policy', 'navdp_Policy'):
        from autonomy_internnav.train.navdp_model import NavDPModelConfig

        return NavDPModelConfig
    if policy_name in ('NavDPGRPO_Policy', 'navdp_grpo_Policy'):
        from autonomy_internnav.train.navdp_grpo_model import NavDPGRPOModelConfig

        return NavDPGRPOModelConfig
    if policy_name == 'InternVLAN1_Policy':
        from autonomy_internnav.internvla_n1.model.internvla_n1_policy import InternVLAN1ModelConfig

        return InternVLAN1ModelConfig
    raise ValueError(f'Policy config for {policy_name!r} not found')
