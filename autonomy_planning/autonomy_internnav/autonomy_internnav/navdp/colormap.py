# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""NavDP critic colormap helpers (overlay projection and RViz markers)."""

from __future__ import annotations

import numpy as np

_JET = None


def _jet_colormap():
    global _JET
    if _JET is None:
        from matplotlib import colormaps as cm
        _JET = cm.get('jet')
    return _JET


def critic_norm_value(value: float) -> float:
    """Normalize critic score for jet colormap (NavDP ``project_trajectory``)."""
    return float(np.clip(-value * 0.1, 0.0, 1.0))


def critic_value_to_rgb(value: float) -> tuple[float, float, float]:
    """Return RGB in [0, 1] using matplotlib jet."""
    rgba = _jet_colormap()(critic_norm_value(value))
    return float(rgba[0]), float(rgba[1]), float(rgba[2])


def critic_value_to_bgr_u8(value: float) -> tuple[int, int, int]:
    """Return BGR uint8 for OpenCV drawing."""
    red, green, blue = critic_value_to_rgb(value)
    return int(blue * 255.0), int(green * 255.0), int(red * 255.0)
