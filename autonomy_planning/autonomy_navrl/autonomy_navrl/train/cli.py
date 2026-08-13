"""Training CLI entry point."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Train navigation policy (autonomy_navrl).')
    parser.add_argument(
        '--config',
        type=str,
        default='',
        help='Path to training YAML config.',
    )
    parser.add_argument(
        '--list-plugins',
        action='store_true',
        help='Print registered robot/task/action/reward/viz plugins and exit.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Validate config and print framework summary without training.',
    )
    parser.add_argument(
        '--backend',
        type=str,
        default='auto',
        choices=['auto', 'isaac', 'mock'],
        help='Simulation backend.',
    )
    parser.add_argument(
        '--device',
        type=str,
        default='',
        help='Override device from config.',
    )
    parser.add_argument(
        '--livestream',
        type=int,
        default=-1,
        choices=[-1, 0, 1, 2],
        help='WebRTC livestream: 0=off, 1=public, 2=local. -1=use config/env.',
    )
    parser.add_argument(
        '--resume',
        type=str,
        default='',
        help='Checkpoint path to resume PPO training.',
    )
    parser.add_argument(
        '--num-envs',
        type=int,
        default=0,
        help='Override parallel env count from config (Isaac vectorized sim).',
    )
    parser.add_argument(
        '--profile',
        type=str,
        default='',
        help='Named profile from config profiles: section (see go2w.yaml / s10.yaml).',
    )
    parser.add_argument(
        '--algorithm',
        type=str,
        default='',
        help='Override algorithm.name from config (ppo | grpo | diffusion).',
    )
    return parser


def _run_training(config: dict[str, Any], backend: str) -> Path:
    from autonomy_navrl.algorithms.factory import create_algorithm, resolve_algorithm_name
    from autonomy_navrl.env.factory import create_training_env

    algo_name = resolve_algorithm_name(config)
    env = create_training_env(config, backend=backend)
    print(f'[INFO]: Training env ready. Starting {algo_name.upper()}...', flush=True)
    try:
        trainer = create_algorithm(env, config, device=str(config.get('device', 'cuda:0')))
        return trainer.train()
    finally:
        env.close()


def main() -> None:
    args = build_arg_parser().parse_args()

    if args.list_plugins:
        from autonomy_navrl.core.api import NavrlFramework
        print(NavrlFramework.plugin_catalog())
        return

    if not args.config:
        build_arg_parser().error('--config is required unless --list-plugins is set')

    from autonomy_navrl.core.config_loader import load_yaml_config
    from autonomy_navrl.core.api import NavrlFramework

    config = load_yaml_config(args.config, profile=args.profile or None)
    if args.device:
        config['device'] = args.device
    if args.livestream >= 0:
        config.setdefault('isaac', {})['livestream'] = args.livestream
    if args.resume:
        config.setdefault('ppo', {})['resume_checkpoint'] = args.resume
    if args.num_envs > 0:
        config['num_envs'] = args.num_envs
    if args.algorithm:
        config.setdefault('algorithm', {})['name'] = args.algorithm

    backend = args.backend
    sim_app = None
    if backend in ('isaac', 'auto') and not args.dry_run:
        try:
            from autonomy_navrl.train.isaac_launch import bootstrap_isaac_app

            sim_app = bootstrap_isaac_app(config)
            if backend == 'auto':
                backend = 'isaac'
        except ImportError as exc:
            if backend == 'isaac':
                raise
            raise ImportError(
                'Isaac Lab is required for visual navigation training. '
                'Run inside the Isaac Lab Docker image with --backend isaac.'
            ) from exc

    framework = NavrlFramework(config)

    if args.dry_run:
        errors = framework.validate()
        print(framework.summary())
        if errors:
            print('[ERROR] Validation failed:')
            for err in errors:
                print(f'  - {err}')
            raise SystemExit(1)
        print('[OK] Config valid.')
        return

    print(framework.summary(), flush=True)
    errors = framework.validate()
    if errors:
        print('[WARN] Config validation:', flush=True)
        for err in errors:
            print(f'  - {err}', flush=True)

    num_envs = int(config.get('num_envs', 1))
    if num_envs >= 16:
        print(
            f'[INFO]: Large-scale training: num_envs={num_envs} '
            f'(rollout batch ≈ rollout_steps × num_envs)',
            flush=True,
        )

    try:
        checkpoint = _run_training(config, backend=backend)
        print(f'Training complete. Checkpoint: {Path(checkpoint).resolve()}')
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        if sim_app is not None:
            sim_app.close()


if __name__ == '__main__':
    main()
