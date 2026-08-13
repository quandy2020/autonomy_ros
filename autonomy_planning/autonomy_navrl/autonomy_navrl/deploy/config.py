"""Deployment configuration helpers."""

from __future__ import annotations

import os
from pathlib import Path

from rclpy.node import Node

from autonomy_navrl.core.types import DeployConfig


def resolve_checkpoint(path: str) -> Path:
    """Resolve checkpoint from parameter, package weights, or env var."""
    env_override = os.environ.get('NAVRL_CHECKPOINT', '').strip()
    if env_override:
        return Path(env_override).expanduser().resolve()

    candidate = Path(path).expanduser()
    if candidate.is_file():
        return candidate.resolve()

    package_weights = Path(__file__).resolve().parents[2] / 'weights' / path
    if package_weights.is_file():
        return package_weights.resolve()

    raise FileNotFoundError(
        f'Checkpoint not found: {path}. Set NAVRL_CHECKPOINT or place '
        f'weights under autonomy_navrl/weights/.'
    )


def load_deploy_config(node: Node) -> DeployConfig:
    """Load deployment config from ROS node parameters."""
    params = {
        name: node.get_parameter(name).value
        for name in DeployConfig.__dataclass_fields__  # type: ignore[attr-defined]
        if node.has_parameter(name)
    }
    config = DeployConfig.from_dict(params)
    config.checkpoint = str(resolve_checkpoint(config.checkpoint))
    return config
