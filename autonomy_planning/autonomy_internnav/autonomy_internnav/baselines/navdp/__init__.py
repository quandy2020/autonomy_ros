"""Backward-compatible re-export. Prefer :mod:`autonomy_internnav.navdp`."""

from autonomy_internnav.navdp.agent import NavDP_Agent
from autonomy_internnav.navdp.policy import NavDP_Policy

__all__ = ['NavDP_Agent', 'NavDP_Policy']
