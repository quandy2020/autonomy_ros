"""Per-model training runners."""

from autonomy_internnav.train.registry import register_runner
from autonomy_internnav.train.runners.navdp import run_navdp_training

register_runner('navdp', run_navdp_training)
