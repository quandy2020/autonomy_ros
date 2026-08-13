"""Plugin packages — call ``ensure_plugins()`` before using registries."""

from __future__ import annotations

from autonomy_navrl.plugins.bootstrap import ensure_plugins

__all__ = ['ensure_plugins']
