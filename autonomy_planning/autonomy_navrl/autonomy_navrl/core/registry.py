"""Lightweight name -> factory registry."""

from __future__ import annotations

from collections.abc import Callable
from typing import Generic, TypeVar

T = TypeVar('T')


class Registry(Generic[T]):
    """Maps string keys to callables that build plugin instances."""

    def __init__(self, label: str) -> None:
        self._label = label
        self._factories: dict[str, Callable[..., T]] = {}

    def register(
        self,
        name: str,
        factory: Callable[..., T] | None = None,
    ) -> Callable[..., T] | Callable[[Callable[..., T]], Callable[..., T]]:
        """Register a factory, optionally as ``@registry.register('name')`` decorator."""

        def decorator(fn: Callable[..., T]) -> Callable[..., T]:
            if name in self._factories:
                raise KeyError(f'{self._label} "{name}" is already registered')
            self._factories[name] = fn
            return fn

        if factory is not None:
            return decorator(factory)
        return decorator

    def create(self, name: str, **kwargs) -> T:
        if name not in self._factories:
            known = ', '.join(sorted(self._factories))
            raise KeyError(f'Unknown {self._label} "{name}". Known: {known}')
        return self._factories[name](**kwargs)

    def names(self) -> list[str]:
        return sorted(self._factories)
