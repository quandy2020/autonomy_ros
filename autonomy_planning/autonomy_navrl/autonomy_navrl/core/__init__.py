"""Isaac Lab RL framework core (specs + registries + public API)."""

from autonomy_navrl.core.api import NavrlFramework
from autonomy_navrl.core.catalog import format_plugin_catalog, framework_summary, list_plugins
from autonomy_navrl.core.config_loader import load_yaml_config
from autonomy_navrl.core.registry import Registry
from autonomy_navrl.core.spec import (
    ACTION_MODELS,
    ROBOT_KINDS,
    TASK_KINDS,
    ActionSpec,
    FrameworkConfig,
    RobotSpec,
    SensorSpec,
    TaskSpec,
)
from autonomy_navrl.core.types import (
    DeployConfig,
    ImuReading,
    OdomReading,
    RgbdFrame,
    TrainConfig,
    VelocityCommand,
)
from autonomy_navrl.core.validate import validate_framework_config

__all__ = [
    'ACTION_MODELS',
    'ROBOT_KINDS',
    'TASK_KINDS',
    'ActionSpec',
    'DeployConfig',
    'FrameworkConfig',
    'ImuReading',
    'NavrlFramework',
    'OdomReading',
    'Registry',
    'RgbdFrame',
    'RobotSpec',
    'SensorSpec',
    'TaskSpec',
    'TrainConfig',
    'VelocityCommand',
    'format_plugin_catalog',
    'framework_summary',
    'list_plugins',
    'load_yaml_config',
    'validate_framework_config',
]
