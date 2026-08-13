"""Public framework API — single entry point for training and extension."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from autonomy_navrl.core.catalog import format_plugin_catalog, framework_summary, list_plugins
from autonomy_navrl.core.spec import FrameworkConfig
from autonomy_navrl.core.validate import validate_framework_config
from autonomy_navrl.env.base_env import BaseNavrlEnv
from autonomy_navrl.env.factory import create_training_env


class NavrlFramework:
    """Load YAML, validate, inspect plugins, and create training environments."""

    def __init__(self, config: dict[str, Any]) -> None:
        self._config = config
        self._framework = FrameworkConfig.from_yaml_dict(config)

    @classmethod
    def from_yaml(cls, path: str | Path, profile: str | None = None) -> NavrlFramework:
        from autonomy_navrl.core.config_loader import load_yaml_config

        return cls(load_yaml_config(str(path), profile=profile))

    @property
    def config(self) -> dict[str, Any]:
        return self._config

    @property
    def framework(self) -> FrameworkConfig:
        return self._framework

    @staticmethod
    def list_plugins() -> dict[str, list[str]]:
        return list_plugins()

    @staticmethod
    def plugin_catalog() -> str:
        return format_plugin_catalog()

    def validate(self) -> list[str]:
        return validate_framework_config(self._config)

    def summary(self) -> str:
        return framework_summary(self._config)

    def create_env(self, backend: str = 'isaac') -> BaseNavrlEnv:
        errors = self.validate()
        if errors:
            raise ValueError('Invalid framework config:\n  - ' + '\n  - '.join(errors))
        return create_training_env(self._config, backend=backend)
