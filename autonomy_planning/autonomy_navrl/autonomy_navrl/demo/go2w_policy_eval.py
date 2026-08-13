"""Evaluate a trained NavRL checkpoint in Isaac Lab (optional WebRTC)."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import torch

from autonomy_navrl.models.checkpoint import load_checkpoint_payload
from autonomy_navrl.models.factory import create_nav_policy
from autonomy_navrl.models.obs import obs_to_torch


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Evaluate trained Go2W NavRL checkpoint in Isaac Sim.')
    parser.add_argument('--config', type=str, required=True, help='YAML config path.')
    parser.add_argument(
        '--checkpoint',
        type=str,
        required=True,
        help='Trained checkpoint (.pt), e.g. checkpoints/navrl_go2w/ckpts/navrl_final.pt',
    )
    parser.add_argument(
        '--profile',
        type=str,
        default='demo',
        help='Config profile (demo=fixed target, webrtc=single env + viz).',
    )
    parser.add_argument('--target-x', type=float, default=None, help='Override target X (m).')
    parser.add_argument('--target-y', type=float, default=None, help='Override target Y (m).')
    parser.add_argument('--target-yaw-deg', type=float, default=None, help='Override target yaw (deg).')
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


def _parse_state(state_row: np.ndarray) -> tuple[float, float, float, float]:
    distance = float(state_row[2])
    yaw_error = float(state_row[4]) if state_row.shape[0] > 4 else 0.0
    speed = float(np.linalg.norm(state_row[17:19])) if state_row.shape[0] > 18 else 0.0
    return distance, yaw_error, speed


def main() -> int:
    args = build_arg_parser().parse_args()
    checkpoint = Path(args.checkpoint)
    if not checkpoint.is_file():
        print(f'[eval] error: checkpoint not found: {checkpoint}', file=sys.stderr, flush=True)
        return 2

    from autonomy_navrl.core.config_loader import load_yaml_config
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
    if args.target_x is not None or args.target_y is not None or args.target_yaw_deg is not None:
        target = list(task_cfg.get('target_pose', [2.0, 0.0, 0.0]))
        if args.target_x is not None:
            target[0] = args.target_x
        if args.target_y is not None:
            target[1] = args.target_y
        if args.target_yaw_deg is not None:
            target[2] = math.radians(args.target_yaw_deg)
        task_cfg['target_pose'] = target
        task_cfg.pop('pose_range', None)

    device = torch.device(str(config.get('device', 'cuda:0')) if torch.cuda.is_available() else 'cpu')
    payload = load_checkpoint_payload(checkpoint, device)
    train_config = payload.get('config', config)
    policy = create_nav_policy(train_config, device=device)
    policy.load_state_dict(payload['policy_state_dict'])
    policy.eval()
    global_step = int(payload.get('global_step', 0))

    print(f'[eval] checkpoint: {checkpoint.resolve()}', flush=True)
    print(f'[eval] global_step: {global_step}', flush=True)
    if 'target_pose' in task_cfg:
        target = task_cfg['target_pose']
        print(
            f'[eval] target: x={target[0]:.2f} y={target[1]:.2f} '
            f'yaw={math.degrees(target[2]):.1f} deg',
            flush=True,
        )

    sim_app = bootstrap_isaac_app(config)
    env = create_training_env(config, backend='isaac')
    try:
        obs = env.reset()
        success = False
        terminated = False
        for step in range(args.max_steps):
            obs_t = obs_to_torch(obs, device)
            with torch.no_grad():
                action, _, _, _ = policy.act(obs_t, deterministic=True)
            result = env.step(action.detach().cpu().numpy())
            obs = result.observation
            metrics = result.info.get('metrics', {})
            distance, yaw_error, speed = _parse_state(obs['state'][0])

            if step % 50 == 0 or result.terminated.any() or result.truncated.any():
                print(
                    f'[eval] step={step:4d} dist={distance:.3f}m speed={speed:.3f}m/s '
                    f'yaw_err={math.degrees(yaw_error):+.1f} deg '
                    f'goal_rate={metrics.get("goal_rate", 0.0):.2f} '
                    f'reward={float(result.reward[0]):.1f}',
                    flush=True,
                )

            if result.terminated.any():
                success = True
                terminated = True
                print(
                    f'[eval] SUCCESS (terminated) at step={step} '
                    f'dist={distance*100:.1f}cm yaw_err={math.degrees(yaw_error):+.2f} deg',
                    flush=True,
                )
                break
            if result.truncated.any():
                print(
                    f'[eval] TIMEOUT at step={step} '
                    f'dist={distance:.3f}m yaw_err={math.degrees(yaw_error):+.1f} deg',
                    flush=True,
                )
                break

        if not terminated and not success:
            print('[eval] finished without success termination', flush=True)
        return 0 if success else 1
    finally:
        env.close()
        sim_app.close()


if __name__ == '__main__':
    sys.exit(main())
