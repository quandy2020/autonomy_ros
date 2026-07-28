"""Console entry points for training."""

from __future__ import annotations

import runpy
from pathlib import Path


def main() -> None:
    from autonomy_internnav.train.train_main import main as train_main

    train_main()


def main_grpo() -> None:
    pkg_root = Path(__file__).resolve().parents[2]
    script = pkg_root / 'scripts' / 'train' / 'train_grpo.py'
    runpy.run_path(str(script), run_name='__main__')
