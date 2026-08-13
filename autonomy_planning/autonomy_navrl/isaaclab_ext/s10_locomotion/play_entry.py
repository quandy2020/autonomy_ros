"""Register S10 velocity env, then delegate to Isaac Lab RSL-RL play.py."""

from __future__ import annotations

import importlib
import os
import runpy
import sys


def main() -> None:
    isaaclab_dir = os.environ.get('ISAACLAB_DIR')
    if not isaaclab_dir:
        raise SystemExit('ISAACLAB_DIR not set')
    train_dir = os.path.join(isaaclab_dir, 'scripts', 'reinforcement_learning', 'rsl_rl')
    if train_dir not in sys.path:
        sys.path.insert(0, train_dir)

    _orig = importlib.import_module

    def _hook(name, package=None):
        mod = _orig(name, package)
        if name == 'isaaclab_tasks':
            import s10_locomotion.register_env  # noqa: F401
        return mod

    importlib.import_module = _hook  # type: ignore[assignment]
    try:
        runpy.run_path(os.path.join(train_dir, 'play.py'), run_name='__main__')
    finally:
        importlib.import_module = _orig  # type: ignore[assignment]


if __name__ == '__main__':
    main()
