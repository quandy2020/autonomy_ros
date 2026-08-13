"""Plugin catalog introspection."""

from __future__ import annotations

from typing import Any


def list_plugins() -> dict[str, list[str]]:
    """Return registered plugin names grouped by category."""
    from autonomy_navrl.plugins.bootstrap import ensure_plugins
    from autonomy_navrl.plugins.actions import action_registry
    from autonomy_navrl.plugins.locomotion.base import locomotion_registry
    from autonomy_navrl.plugins.rewards import reward_registry
    from autonomy_navrl.core.spec import ROBOT_KINDS
    from autonomy_navrl.plugins.robots import ROBOT_PRESETS
    from autonomy_navrl.plugins.tasks import task_registry
    from autonomy_navrl.viz.isaac.config import MODULE_REGISTRY_KEYS

    ensure_plugins()

    return {
        'robot_kinds': sorted(ROBOT_KINDS),
        'robot_presets': sorted(ROBOT_PRESETS),
        'action_models': action_registry.names(),
        'task_kinds': task_registry.names(),
        'reward_profiles': reward_registry.names(),
        'locomotion_backends': locomotion_registry.names(),
        'viz_modules': sorted(MODULE_REGISTRY_KEYS),
    }


def format_plugin_catalog() -> str:
    """Human-readable plugin listing for CLI / logs."""
    catalog = list_plugins()
    lines = ['autonomy_navrl plugin catalog:']
    for category, names in catalog.items():
        lines.append(f'  {category}: {", ".join(names)}')
    return '\n'.join(lines)


def framework_summary(config: dict[str, Any]) -> str:
    """One-line + multi-line summary of a training YAML."""
    from autonomy_navrl.core.spec import FrameworkConfig

    fw = FrameworkConfig.from_yaml_dict(config)
    lines = [
        'Framework configuration:',
        f'  robot     : kind={fw.robot.kind} preset={config.get("robot", {}).get("preset", "-")}',
        f'  action    : {fw.action.model} (dim={fw.action.dim})',
        f'  task      : {fw.task.kind}',
        f'  sensors   : rgb={fw.sensors.rgb_width}x{fw.sensors.rgb_height} '
        f'imu={fw.sensors.imu_enabled} odom={fw.sensors.odom_enabled}',
        f'  reward    : {fw.reward_profile}',
        f'  locomotion: {fw.locomotion.backend}',
        f'  num_envs  : {config.get("num_envs", 1)}',
        f'  backend   : isaac (use --backend mock for CI)',
    ]
    viz = fw.viz_raw or {}
    if viz.get('enabled'):
        modules = viz.get('modules') or []
        lines.append(f'  viz       : {", ".join(modules)}')
    return '\n'.join(lines)
