"""Isaac Lab environment modules (lazy imports)."""

from __future__ import annotations

from typing import TYPE_CHECKING

__all__ = [
    'NavrlDirectEnv',
    'NavrlEnvCfg',
    'build_navrl_cfg',
]

if TYPE_CHECKING:
    from autonomy_navrl.env.isaac.cfg import NavrlEnvCfg, build_navrl_cfg
    from autonomy_navrl.env.isaac.direct_env import NavrlDirectEnv


def __getattr__(name: str):
    if name in ('NavrlEnvCfg', 'build_navrl_cfg'):
        from autonomy_navrl.env.isaac.cfg import NavrlEnvCfg, build_navrl_cfg

        return {'NavrlEnvCfg': NavrlEnvCfg, 'build_navrl_cfg': build_navrl_cfg}[name]
    if name == 'NavrlDirectEnv':
        from autonomy_navrl.env.isaac.direct_env import NavrlDirectEnv

        return NavrlDirectEnv
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
