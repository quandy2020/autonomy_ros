"""S10 locomotion smoke test — constant velocity through s10_gait / stance / JIT."""

from __future__ import annotations

import argparse
import sys

import numpy as np


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='S10 locomotion smoke test.')
    parser.add_argument('--config', type=str, required=True)
    parser.add_argument('--profile', type=str, default='s10-loco')
    parser.add_argument('--vx', type=float, default=0.3, help='Forward speed (m/s).')
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--w', type=float, default=0.0, help='Yaw rate (rad/s).')
    parser.add_argument('--max-steps', type=int, default=500)
    parser.add_argument(
        '--gait-mode',
        type=str,
        default='',
        choices=['', 'auto', 'stand', 'march', 'walk', 'strafe'],
        help='Force s10_gait mode (default: config / auto).',
    )
    parser.add_argument(
        '--hold-stance',
        action='store_true',
        help='PD-hold default standing pose; freeze gait / JIT.',
    )
    parser.add_argument('--headless', action='store_true')
    parser.add_argument(
        '--livestream',
        type=int,
        default=-1,
        choices=[-1, 0, 1, 2],
        help='WebRTC livestream: 0=off, 1=public, 2=local. -1=use config/env.',
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    from autonomy_navrl.core.config_loader import load_yaml_config
    from autonomy_navrl.env.factory import create_training_env
    from autonomy_navrl.train.isaac_launch import bootstrap_isaac_app

    config = load_yaml_config(args.config, profile=args.profile or None)
    if args.headless:
        config['headless'] = True
    if args.livestream >= 0:
        config.setdefault('isaac', {})['livestream'] = args.livestream
    loco = config.setdefault('locomotion', {})
    if args.hold_stance:
        loco['hold_stance'] = True
        print('[s10-loco] HOLD STANCE: legs locked at default pose', flush=True)
    if args.gait_mode:
        loco.setdefault('gait', {})['mode'] = args.gait_mode
        loco['gait_mode'] = args.gait_mode
        print(f'[s10-loco] gait_mode={args.gait_mode}', flush=True)

    control = config.get('control', {})
    max_vx = float(control.get('max_vx', 0.5))
    max_vy = float(control.get('max_vy', 0.3))
    max_w = float(control.get('max_w', 1.0))

    vx_n = np.clip(args.vx / max(max_vx, 1e-6), -1.0, 1.0)
    vy_n = np.clip(args.vy / max(max_vy, 1e-6), -1.0, 1.0)
    w_n = np.clip(args.w / max(max_w, 1e-6), -1.0, 1.0)
    action = np.array([vx_n, vy_n, w_n], dtype=np.float32)

    print(
        f'[s10-loco] backend={loco.get("backend")} '
        f'cmd=({args.vx:.2f},{args.vy:.2f},{args.w:.2f}) '
        f'norm=({vx_n:.2f},{vy_n:.2f},{w_n:.2f}) steps={args.max_steps}',
        flush=True,
    )

    sim_app = bootstrap_isaac_app(config)
    env = create_training_env(config, backend='isaac')
    try:
        obs = env.reset()
        step = 0
        while step < args.max_steps:
            result = env.step(np.tile(action, (env.num_envs, 1)))
            obs = result.observation
            if step % 50 == 0:
                speed = float(np.linalg.norm(obs['state'][0, 17:19]))
                robot = env.direct_env._robot
                z = float(robot.data.root_pos_w[0, 2])
                grav_z = float(robot.data.projected_gravity_b[0, 2])
                print(
                    f'[s10-loco] step={step:4d} body_speed≈{speed:.3f}m/s '
                    f'z={z:.3f} grav_z={grav_z:.3f}',
                    flush=True,
                )
            step += 1
            if result.truncated.any() or result.terminated.any():
                obs = env.reset()
        print('[s10-loco] done', flush=True)
        return 0
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        env.close()
        sim_app.close()


if __name__ == '__main__':
    sys.exit(main())
