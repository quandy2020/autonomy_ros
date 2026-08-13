"""Training pipeline for navigation DRL."""

from __future__ import annotations

__all__ = ['PpoAlgorithm']


def __getattr__(name: str):
    if name == 'PpoAlgorithm':
        from autonomy_navrl.algorithms.ppo import PpoAlgorithm
        return PpoAlgorithm
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
