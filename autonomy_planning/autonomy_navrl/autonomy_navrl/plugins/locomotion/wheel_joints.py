"""Shared wheeled-legged joint name helpers (Go2W / S10)."""

from __future__ import annotations

GO2W_WHEEL_JOINT_NAMES: tuple[str, ...] = (
    'FR_foot_joint', 'FL_foot_joint', 'RR_foot_joint', 'RL_foot_joint',
)

S10_WHEEL_JOINT_NAMES: tuple[str, ...] = (
    'fl_wheel_joint', 'fr_wheel_joint', 'hl_wheel_joint', 'hr_wheel_joint',
)

WHEELED_LEG_WHEEL_CANDIDATES: tuple[tuple[str, ...], ...] = (
    GO2W_WHEEL_JOINT_NAMES,
    S10_WHEEL_JOINT_NAMES,
)


def resolve_joint_indices(joint_names: list[str], expected: tuple[str, ...]) -> list[int]:
    name_to_index = {name: index for index, name in enumerate(joint_names)}
    missing = [name for name in expected if name not in name_to_index]
    if missing:
        raise RuntimeError(
            f'Missing joints {missing}. Available: {joint_names}'
        )
    return [name_to_index[name] for name in expected]


def resolve_wheel_joint_indices(joint_names: list[str]) -> list[int] | None:
    for expected in WHEELED_LEG_WHEEL_CANDIDATES:
        try:
            return resolve_joint_indices(joint_names, expected)
        except RuntimeError:
            continue
    return None
