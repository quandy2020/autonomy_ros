"""NavDP critic colormap helpers (shared by policy overlay and RViz markers)."""

from __future__ import annotations

import numpy as np

_JET = None


def _jet():
    global _JET
    if _JET is None:
        from matplotlib import colormaps as cm
        _JET = cm.get('jet')
    return _JET


def critic_norm_value(value: float) -> float:
    """NavDP ``project_trajectory`` normalization."""
    return float(np.clip(-value * 0.1, 0.0, 1.0))


def critic_value_to_rgb(value: float) -> tuple[float, float, float]:
    """Return RGB in [0, 1] using matplotlib jet (matches NavDP overlay)."""
    rgba = _jet()(critic_norm_value(value))
    return float(rgba[0]), float(rgba[1]), float(rgba[2])


def critic_value_to_bgr_u8(value: float) -> tuple[int, int, int]:
    """Return BGR uint8 for OpenCV drawing (matches NavDP ``project_trajectory``)."""
    r, g, b = critic_value_to_rgb(value)
    return int(b * 255.0), int(g * 255.0), int(r * 255.0)
