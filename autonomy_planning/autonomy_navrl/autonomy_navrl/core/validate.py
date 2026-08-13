"""Configuration validation helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def validate_framework_config(config: dict[str, Any]) -> list[str]:
    """Return a list of validation errors (empty if OK)."""
    errors: list[str] = []

    try:
        from autonomy_navrl.core.spec import FrameworkConfig

        FrameworkConfig.from_yaml_dict(config)
    except ValueError as exc:
        errors.append(str(exc))
        return errors

    urdf = str(config.get('robot', {}).get('urdf_path', '')).strip()
    if urdf and not urdf.startswith('/dev/null') and not Path(urdf).exists():
        errors.append(f'robot.urdf_path does not exist: {urdf}')

    state_dim = int(config.get('policy', {}).get('state_dim', 19))
    task_kind = str(config.get('task', {}).get('kind') or config.get('task', {}).get('mode', 'goal_nav'))
    if task_kind in ('precision_pose', 'go2w_precision_pose', 'navigate', 'goal_nav') and state_dim < 12:
        errors.append(f'policy.state_dim={state_dim} may be too small for task {task_kind}')

    action_dim = int(config.get('control', {}).get('action_dim', 3))
    action_model = config.get('control', {}).get('action_model')
    if action_model == 'diff_drive_2d' and action_dim not in (0, 2):
        errors.append('diff_drive_2d expects control.action_dim=2')

    ppo = config.get('ppo', {})
    rollout = int(ppo.get('rollout_steps', 24))
    num_envs = int(config.get('num_envs', 1))
    if rollout * num_envs < num_envs:
        errors.append('ppo.rollout_steps must be >= 1')

    return errors
