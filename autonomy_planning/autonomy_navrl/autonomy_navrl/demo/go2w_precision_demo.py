"""Go2W precision pose alignment demo in Isaac Lab."""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Go2W precision pose alignment demo.')
    parser.add_argument('--config', type=str, required=True, help='Demo YAML config path.')
    parser.add_argument(
        '--profile',
        type=str,
        default='demo',
        help='Profile name when config uses profiles: section (default: demo).',
    )
    parser.add_argument('--target-x', type=float, default=None, help='Target X (m).')
    parser.add_argument('--target-y', type=float, default=None, help='Target Y (m).')
    parser.add_argument('--target-yaw-deg', type=float, default=None, help='Target yaw (deg).')
    parser.add_argument('--max-steps', type=int, default=3000, help='Max simulation steps.')
    parser.add_argument('--headless', action='store_true', help='Run Isaac headless.')
    parser.add_argument(
        '--livestream',
        type=int,
        default=-1,
        choices=[-1, 0, 1, 2],
        help='WebRTC livestream: 0=off, 1=public, 2=local. -1=use config/env.',
    )
    return parser


def _parse_state(state_row: np.ndarray) -> tuple[float, float, float, float, float, float]:
    """Extract pose-tracking features from env state vector."""
    dx_body = float(state_row[0])
    dy_body = float(state_row[1])
    distance = float(state_row[2])
    bearing = float(state_row[3])
    yaw_error = float(state_row[4]) if state_row.shape[0] > 4 else 0.0
    speed = float(np.linalg.norm(state_row[17:19])) if state_row.shape[0] > 18 else 0.0
    return dx_body, dy_body, distance, bearing, yaw_error, speed


def main() -> int:
    args = build_arg_parser().parse_args()
    from autonomy_navrl.core.config_loader import load_yaml_config
    from autonomy_navrl.control.precision_controller import (
        compute_pose_tracking_action,
        is_fully_aligned,
    )
    from autonomy_navrl.env.factory import create_training_env
    from autonomy_navrl.train.isaac_launch import bootstrap_isaac_app

    config = load_yaml_config(args.config, profile=args.profile or None)
    if args.headless:
        config['headless'] = True
    if args.livestream >= 0:
        config.setdefault('isaac', {})['livestream'] = args.livestream
        if args.livestream > 0:
            config.setdefault('viz', {})['enabled'] = True
            config['viz'].setdefault('auto_enable_on_livestream', True)

    task_cfg = config.setdefault('task', {})
    target = list(task_cfg.get('target_pose', [2.0, 0.0, 0.0]))
    if args.target_x is not None:
        target[0] = args.target_x
    if args.target_y is not None:
        target[1] = args.target_y
    if args.target_yaw_deg is not None:
        target[2] = math.radians(args.target_yaw_deg)
    task_cfg['target_pose'] = target
    task_cfg['mode'] = 'precision_pose'

    control_cfg = config.get('control', {})
    max_vx = float(control_cfg.get('max_vx', 0.5))
    max_vy = float(control_cfg.get('max_vy', 0.5))
    max_w = float(control_cfg.get('max_w', 1.0))
    pos_tol = float(task_cfg.get('position_tolerance_m', task_cfg.get('goal_tolerance_m', 0.05)))
    yaw_tol = float(task_cfg.get('yaw_tolerance_rad', 0.1))
    speed_tol = float(task_cfg.get('require_stop_speed_mps', 0.08))

    print(
        f'[demo] Go2W full alignment target: x={target[0]:.3f} y={target[1]:.3f} '
        f'yaw={math.degrees(target[2]):.1f} deg',
        flush=True,
    )
    print(
        f'[demo] tolerances: pos={pos_tol*100:.1f}cm '
        f'yaw={math.degrees(yaw_tol):.1f} deg speed<{speed_tol:.2f}m/s',
        flush=True,
    )

    sim_app = bootstrap_isaac_app(config)
    env = create_training_env(config, backend='isaac')
    try:
        obs = env.reset()
        success = False
        for step in range(args.max_steps):
            state = obs['state']
            dx_body, dy_body, distance, bearing, yaw_error, speed = _parse_state(state[0])

            action = compute_pose_tracking_action(
                dx_body,
                dy_body,
                distance,
                bearing,
                yaw_error,
                max_vx=max_vx,
                max_vy=max_vy,
                max_w=max_w,
            )
            action_batch = np.tile(action, (env.num_envs, 1))
            result = env.step(action_batch)
            obs = result.observation
            metrics = result.info.get('metrics', {})

            if step % 50 == 0 or result.terminated.any() or result.truncated.any():
                print(
                    f'[demo] step={step:4d} dist={distance:.3f}m speed={speed:.3f}m/s '
                    f'bearing={math.degrees(bearing):+.1f} deg '
                    f'yaw_err={math.degrees(yaw_error):+.1f} deg '
                    f'goal_rate={metrics.get("goal_rate", 0.0):.2f}',
                    flush=True,
                )

            aligned = is_fully_aligned(
                distance,
                yaw_error,
                speed,
                pos_tol_m=pos_tol,
                yaw_tol_rad=yaw_tol,
                speed_tol_mps=speed_tol,
            )
            if result.terminated.any() or aligned:
                success = aligned or result.terminated.any()
                print(
                    f'[demo] SUCCESS fully aligned at step={step} '
                    f'dist={distance*100:.1f}cm '
                    f'yaw_err={math.degrees(yaw_error):+.2f} deg '
                    f'speed={speed:.3f}m/s',
                    flush=True,
                )
                break
            if result.truncated.any():
                print(
                    f'[demo] TIMEOUT at step={step} '
                    f'dist={distance:.3f}m yaw_err={math.degrees(yaw_error):+.1f} deg',
                    flush=True,
                )
                break
        return 0 if success else 1
    finally:
        env.close()
        sim_app.close()


if __name__ == '__main__':
    sys.exit(main())
