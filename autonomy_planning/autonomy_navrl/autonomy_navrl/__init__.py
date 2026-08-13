"""autonomy_navrl — Isaac Lab modular RL training framework."""

from autonomy_navrl.core.api import NavrlFramework
from autonomy_navrl.core.catalog import list_plugins
from autonomy_navrl.env.factory import create_training_env

__version__ = '0.4.0'

__all__ = [
    'NavrlFramework',
    '__version__',
    'create_training_env',
    'list_plugins',
]
