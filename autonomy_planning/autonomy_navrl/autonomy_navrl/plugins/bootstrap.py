"""Built-in plugin registration (idempotent)."""

from __future__ import annotations

_registered = False


def ensure_plugins() -> None:
    """Import plugin modules once so @registry.register side effects run."""
    global _registered
    if _registered:
        return

    from autonomy_navrl.plugins import actions as _actions  # noqa: F401
    # isaac_sensors imports isaaclab.sensors and requires AppLauncher first;
    # loaded lazily from env/isaac/* after bootstrap_isaac_app().
    from autonomy_navrl.plugins import robots as _robots  # noqa: F401
    from autonomy_navrl.plugins import tasks as _tasks  # noqa: F401
    from autonomy_navrl.plugins import rewards as _rewards  # noqa: F401
    from autonomy_navrl.plugins import rewards_extras as _rewards_extras  # noqa: F401
    from autonomy_navrl.plugins.locomotion import go2w_jit as _go2w_jit  # noqa: F401
    from autonomy_navrl.plugins.locomotion import s10_jit as _s10_jit  # noqa: F401
    from autonomy_navrl.plugins.locomotion import s10_stance_drive as _s10_stance_drive  # noqa: F401
    from autonomy_navrl.plugins.locomotion import s10_gait as _s10_gait  # noqa: F401
    from autonomy_navrl.plugins.locomotion import root_velocity as _root_velocity  # noqa: F401
    from autonomy_navrl.plugins.go2w import rewards as _go2w_rewards  # noqa: F401
    from autonomy_navrl.plugins.go2w import task as _go2w_task  # noqa: F401

    _registered = True
