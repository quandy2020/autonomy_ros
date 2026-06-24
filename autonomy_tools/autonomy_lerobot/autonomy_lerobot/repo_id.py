"""LeRobot dataset repo_id helpers."""

from __future__ import annotations


def sanitize_repo_id(repo_id: str) -> str:
    """LeRobot repo_id must be ``namespace/repo_name`` (exactly one slash)."""
    repo_id = repo_id.strip()
    if not repo_id:
        return 'local/habitat_collection'
    parts = [p for p in repo_id.split('/') if p]
    if len(parts) == 1:
        return f'local/{parts[0]}'
    if len(parts) == 2:
        return f'{parts[0]}/{parts[1]}'
    return f'{parts[0]}/{"_".join(parts[1:])}'


def per_robot_repo_id(base: str, robot: str) -> str:
    """Derive a valid per-robot repo_id from a shared collection id."""
    base = base.strip()
    robot = robot.strip('/')
    if '/' in base:
        namespace, name = base.split('/', 1)
        return sanitize_repo_id(f'{namespace}/{name}_{robot}')
    return sanitize_repo_id(f'local/{base}_{robot}')
