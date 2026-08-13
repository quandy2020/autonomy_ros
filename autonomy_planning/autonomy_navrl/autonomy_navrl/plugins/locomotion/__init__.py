"""Locomotion controller plugins (registration via ``plugins.bootstrap``)."""

from autonomy_navrl.plugins.locomotion.base import (
    BaseLocomotionController,
    create_locomotion_controller,
    locomotion_registry,
)

__all__ = [
    'BaseLocomotionController',
    'create_locomotion_controller',
    'locomotion_registry',
]
