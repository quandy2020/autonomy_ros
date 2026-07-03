"""Backward-compatible re-exports. Prefer ``autonomy_internnav.baselines.navdp``."""

from autonomy_internnav.baselines.navdp.policy_agent import NavDP_Agent
from autonomy_internnav.baselines.navdp.policy_network import NavDP_Policy

__all__ = ['NavDP_Agent', 'NavDP_Policy']
