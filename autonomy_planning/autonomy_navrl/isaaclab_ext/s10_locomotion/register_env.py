"""Register S10 velocity env with Isaac Lab gym registry."""

from __future__ import annotations

import gymnasium as gym

# Do not import s10_velocity_env_cfg here — Isaac Sim (omni.*) is only available
# after AppLauncher starts; cfg is loaded lazily via env_cfg_entry_point strings.

gym.register(
    id='Isaac-Velocity-Rough-S10-v0',
    entry_point='isaaclab.envs:ManagerBasedRLEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': f'{__name__.split(".")[0]}.s10_velocity_env_cfg:S10RoughEnvCfg',
        'rsl_rl_cfg_entry_point': f'{__name__.split(".")[0]}.agents.rsl_rl_ppo_cfg:S10RoughPPORunnerCfg',
    },
)

gym.register(
    id='Isaac-Velocity-Rough-S10-Play-v0',
    entry_point='isaaclab.envs:ManagerBasedRLEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': f'{__name__.split(".")[0]}.s10_velocity_env_cfg:S10RoughEnvCfg_PLAY',
        'rsl_rl_cfg_entry_point': f'{__name__.split(".")[0]}.agents.rsl_rl_ppo_cfg:S10RoughPPORunnerCfg',
    },
)
