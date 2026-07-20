#!/usr/bin/env python3
#
# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

"""Minimal interactive Habitat-Sim viewer for scene and robot inspection."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from habitat.config import Config
from habitat.robot.avatar import RobotAvatar
from habitat.robot.paths import materialize_robot_urdf, robot_type_metadata
from habitat.sim import Session


class _Logger:
    def info(self, msg: str) -> None:
        print(f'[INFO] {msg}')

    def warning(self, msg: str) -> None:
        print(f'[WARN] {msg}')

    def error(self, msg: str) -> None:
        print(f'[ERROR] {msg}')


def _default_dataset_config(repo_root: Path, mp3d_root: Path) -> tuple[str, str]:
    scene_dataset = mp3d_root / 'mp3d.scene_dataset_config.json'
    package_dataset = (
        repo_root
        / 'src'
        / 'autonomy_ros'
        / 'autonomy_simulator'
        / 'configs'
        / 'mp3d.scene_dataset_config.json'
    )
    return (str(scene_dataset), str(package_dataset))


def _make_cfg(args: argparse.Namespace) -> Config:
    scene_dir = Path(args.scene_dir).resolve()
    repo_root = Path(__file__).resolve().parents[4]
    scene_dataset, package_dataset = _default_dataset_config(repo_root, scene_dir.parent)
    return Config(
        scene_data_path=str(scene_dir),
        scene_id=scene_dir.name,
        mp3d_root=str(scene_dir.parent),
        scene_dataset_config=scene_dataset,
        package_scene_dataset_config=package_dataset,
        image_width=int(args.width),
        image_height=int(args.height),
        camera_horizontal_fov_deg=float(args.hfov),
        sensor_height=float(args.sensor_height),
        spawn_mode='fixed',
        spawn_x=float(args.spawn_x),
        spawn_y=float(args.spawn_y),
        spawn_yaw=float(args.spawn_yaw),
        pedestrians_enabled=bool(args.robot_assets),
        pedestrian_count=max(1, len(args.robot_assets)),
        dynamic_actor_kind='robot',
        robot_asset_root=str(args.robot_asset_root or ''),
        robot_asset_type=(args.robot_assets[0] if args.robot_assets else 'spot'),
        robot_asset_types=','.join(args.robot_assets),
        robot_agent_count=len(args.robot_assets),
        topdown_enabled=False,
        update_rate_hz=30.0,
    )


def _spawn_robot_positions(session: Session, count: int) -> list[np.ndarray]:
    pf = session.pathfinder
    if pf is None or not pf.is_loaded or count <= 0:
        return []
    base_x, base_y, _, _ = session.map_pose()
    points: list[np.ndarray] = []
    for idx in range(count):
        angle = idx * (2.0 * math.pi / max(count, 1))
        radius = 1.5 + 0.6 * idx
        guess = np.array(
            [base_x + radius * math.cos(angle), session.floor_height, -(base_y + radius * math.sin(angle))],
            dtype=np.float32,
        )
        point = session._snap(guess)  # noqa: SLF001
        points.append(point)
    return points


def _spawn_robots(cfg: Config, session: Session, robot_assets: list[str], logger: _Logger) -> list[RobotAvatar]:
    robots: list[RobotAvatar] = []
    positions = _spawn_robot_positions(session, len(robot_assets))
    for idx, asset_name in enumerate(robot_assets):
        urdf = materialize_robot_urdf(cfg, asset_name)
        semantic_id = int(robot_type_metadata(cfg, asset_name)['semantic_id'])
        robot = RobotAvatar(session.sim, urdf, semantic_id, asset_name, logger)
        robot.spawn()
        point = positions[idx] if idx < len(positions) else np.array([0.0, session.floor_height, 0.0], dtype=np.float32)
        yaw = idx * 0.7
        robot.set_base_from_habitat(point, yaw)
        robots.append(robot)
        logger.info(f'viewer spawned robot[{idx}] asset={asset_name} urdf={urdf}')
    return robots


def _render_bgr(obs: dict[str, np.ndarray]) -> np.ndarray:
    frame = obs['color_sensor']
    if frame.ndim == 3 and frame.shape[2] == 4:
        frame = frame[:, :, :3]
    frame = np.ascontiguousarray(frame)
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def _draw_help(frame: np.ndarray, robot_assets: list[str]) -> np.ndarray:
    lines = [
        'Habitat Viewer',
        'W/S: forward/back',
        'A/D: strafe left/right',
        'Q/E: yaw left/right',
        'R/F: camera up/down',
        'ESC: quit',
        f'robots: {", ".join(robot_assets) if robot_assets else "none"}',
    ]
    out = frame.copy()
    y = 24
    for line in lines:
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 240, 40), 1, cv2.LINE_AA)
        y += 22
    return out


def _apply_key(session: Session, key: int, step_xy: float, step_yaw: float, step_z: float) -> bool:
    x, y, _, yaw = session.map_pose()
    floor = float(session.floor_height)
    if key == 27:
        return False
    if key == ord('w'):
        x += step_xy * math.cos(yaw)
        y += step_xy * math.sin(yaw)
    elif key == ord('s'):
        x -= step_xy * math.cos(yaw)
        y -= step_xy * math.sin(yaw)
    elif key == ord('a'):
        x += step_xy * math.cos(yaw + math.pi / 2.0)
        y += step_xy * math.sin(yaw + math.pi / 2.0)
    elif key == ord('d'):
        x += step_xy * math.cos(yaw - math.pi / 2.0)
        y += step_xy * math.sin(yaw - math.pi / 2.0)
    elif key == ord('q'):
        yaw += step_yaw
    elif key == ord('e'):
        yaw -= step_yaw
    elif key == ord('r'):
        floor += step_z
    elif key == ord('f'):
        floor = max(0.05, floor - step_z)
    else:
        return True
    session._floor_height = floor  # noqa: SLF001
    session._move_map(x, y, yaw)  # noqa: SLF001
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Minimal interactive Habitat-Sim viewer.')
    parser.add_argument(
        '--scene-dir',
        default='/mnt/data4t/Datasets/mp3d/17DRP5sb8fy',
        help='Scene directory containing <scene_id>.glb and optional .navmesh',
    )
    parser.add_argument(
        '--robot-assets',
        default='spot',
        help='Comma-separated robot assets to spawn, e.g. spot,jackal,husky',
    )
    parser.add_argument('--robot-asset-root', default='', help='Optional robot asset root override')
    parser.add_argument('--width', type=int, default=960, help='Viewer image width')
    parser.add_argument('--height', type=int, default=540, help='Viewer image height')
    parser.add_argument('--hfov', type=float, default=90.0, help='Camera horizontal FOV in degrees')
    parser.add_argument('--sensor-height', type=float, default=0.9, help='Camera sensor height')
    parser.add_argument('--spawn-x', type=float, default=0.0, help='Initial map x')
    parser.add_argument('--spawn-y', type=float, default=0.0, help='Initial map y')
    parser.add_argument('--spawn-yaw', type=float, default=0.0, help='Initial map yaw in radians')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.robot_assets = [item.strip() for item in str(args.robot_assets).split(',') if item.strip()]
    logger = _Logger()
    cfg = _make_cfg(args)
    session = Session(cfg, logger)
    robots = _spawn_robots(cfg, session, args.robot_assets, logger)
    cv2.namedWindow('Habitat Viewer', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Habitat Viewer', int(args.width), int(args.height))
    running = True
    try:
        while running:
            obs = session.observe()
            frame = _draw_help(_render_bgr(obs), args.robot_assets)
            cv2.imshow('Habitat Viewer', frame)
            key = cv2.waitKey(16) & 0xFF
            running = _apply_key(session, key, step_xy=0.18, step_yaw=0.14, step_z=0.08)
    finally:
        for robot in robots:
            robot.remove()
        session.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
