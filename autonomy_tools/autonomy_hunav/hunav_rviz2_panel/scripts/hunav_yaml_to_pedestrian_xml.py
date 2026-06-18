#!/usr/bin/env python3
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# SPDX-License-Identifier: Apache-2.0

"""Convert a HuNav scenario YAML (hunav_loader format) to autonomy_pedestrian XML."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml


def _load_params(yaml_path: Path) -> Dict[str, Any]:
    with yaml_path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not data or 'hunav_loader' not in data:
        raise ValueError(f'Missing hunav_loader section in {yaml_path}')
    params = data['hunav_loader'].get('ros__parameters', {})
    if not params:
        raise ValueError(f'Missing ros__parameters in {yaml_path}')
    return params


def _global_goals(params: Dict[str, Any]) -> Dict[int, Tuple[float, float]]:
    raw = params.get('global_goals') or {}
    goals: Dict[int, Tuple[float, float]] = {}
    for key, value in raw.items():
        gid = int(key)
        goals[gid] = (float(value['x']), float(value['y']))
    return goals


def _agent_goal_points(agent: Dict[str, Any], goals: Dict[int, Tuple[float, float]]) -> List[Tuple[float, float]]:
    points: List[Tuple[float, float]] = []
    for gid in agent.get('goals') or []:
        if int(gid) not in goals:
            raise KeyError(f'Agent references unknown global goal id {gid}')
        points.append(goals[int(gid)])
    return points


def convert(yaml_path: Path, xml_path: Path) -> None:
    params = _load_params(yaml_path)
    goals = _global_goals(params)
    agent_names: List[str] = list(params.get('agents') or [])
    if not agent_names:
        raise ValueError('No agents listed in scenario YAML')

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<!-- Generated from HuNav scenario YAML -->',
        '<tag type="gaussian"/>',
    ]

    for name in agent_names:
        if name not in params:
            raise KeyError(f'Agent block {name!r} missing from YAML')
        agent = params[name]
        init_pose = agent.get('init_pose') or {}
        sx = float(init_pose.get('x', 0.0))
        sy = float(init_pose.get('y', 0.0))
        path_points = _agent_goal_points(agent, goals)
        if not path_points:
            raise ValueError(f'Agent {name} has no goals')

        lines.append('<pedestrian>')
        lines.append(f'  <start x="{sx:.6f}" y="{sy:.6f}"/>')
        lines.append('  <path>')
        for px, py in path_points:
            lines.append(f'    <point x="{px:.6f}" y="{py:.6f}"/>')
        lines.append('  </path>')
        lines.append('</pedestrian>')

    xml_path.parent.mkdir(parents=True, exist_ok=True)
    xml_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('yaml_path', type=Path, help='HuNav scenario YAML file')
    parser.add_argument(
        'xml_path',
        type=Path,
        nargs='?',
        help='Output XML path (prints path to stdout when omitted uses sibling .xml)',
    )
    args = parser.parse_args()

    yaml_path = args.yaml_path.expanduser().resolve()
    if not yaml_path.is_file():
        print(f'YAML not found: {yaml_path}', file=sys.stderr)
        return 1

    xml_path = args.xml_path
    if xml_path is None:
        xml_path = yaml_path.with_suffix('.xml')
    else:
        xml_path = xml_path.expanduser().resolve()

    convert(yaml_path, xml_path)
    print(str(xml_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
