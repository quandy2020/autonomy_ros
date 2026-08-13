"""Go2W batch demo: random robot spawn × N trials with Isaac viewport MP4 export."""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path

import numpy as np


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Go2W batch alignment demo with video export.')
    parser.add_argument('--config', type=str, required=True)
    parser.add_argument('--profile', type=str, default='demo')
    parser.add_argument('--trials', type=int, default=5, help='Number of random-spawn trials.')
    parser.add_argument('--seed', type=int, default=0, help='RNG seed for spawn sampling.')
    parser.add_argument('--target-x', type=float, default=None)
    parser.add_argument('--target-y', type=float, default=None)
    parser.add_argument('--target-yaw-deg', type=float, default=None)
    parser.add_argument('--spawn-x-min', type=float, default=-1.5)
    parser.add_argument('--spawn-x-max', type=float, default=1.5)
    parser.add_argument('--spawn-y-min', type=float, default=-1.5)
    parser.add_argument('--spawn-y-max', type=float, default=1.5)
    parser.add_argument('--spawn-yaw-min-deg', type=float, default=-180.0)
    parser.add_argument('--spawn-yaw-max-deg', type=float, default=180.0)
    parser.add_argument('--max-steps', type=int, default=3000)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--livestream', type=int, default=-1, choices=[-1, 0, 1, 2])
    parser.add_argument(
        '--video-dir',
        type=str,
        default='',
        help='Directory for MP4 files (default: checkpoints/go2w_batch_demo/videos/<timestamp>).',
    )
    parser.add_argument('--video-fps', type=int, default=20)
    parser.add_argument(
        '--video-source',
        type=str,
        default='viewport',
        choices=['viewport', 'onboard'],
        help='Record Isaac scene viewport (default) or robot onboard RGB camera.',
    )
    parser.add_argument(
        '--video-scale',
        type=int,
        default=1,
        help='Upscale exported frames (viewport default 1, onboard often 4).',
    )
    parser.add_argument('--no-video', action='store_true', help='Skip MP4 export.')
    return parser


def _parse_state(state_row: np.ndarray) -> tuple[float, float, float, float, float, float]:
    dx_body = float(state_row[0])
    dy_body = float(state_row[1])
    distance = float(state_row[2])
    bearing = float(state_row[3])
    yaw_error = float(state_row[4]) if state_row.shape[0] > 4 else 0.0
    speed = float(np.linalg.norm(state_row[17:19])) if state_row.shape[0] > 18 else 0.0
    return dx_body, dy_body, distance, bearing, yaw_error, speed


def _rgbd_to_bgr_frame(rgbd: np.ndarray, scale: int) -> np.ndarray:
    from autonomy_navrl.viz.isaac.viewport_capture import rgb_frame_to_bgr

    rgb = np.clip(rgbd[:3].transpose(1, 2, 0), 0.0, 1.0)
    frame = (rgb * 255.0).astype(np.uint8)
    return rgb_frame_to_bgr(frame, scale=scale)


def _capture_frame(env, obs: dict[str, np.ndarray], *, source: str, scale: int) -> np.ndarray | None:
    if source == 'viewport':
        from autonomy_navrl.viz.isaac.viewport_capture import capture_viewport_bgr

        return capture_viewport_bgr(env, scale=scale)
    return _rgbd_to_bgr_frame(obs['rgbd'][0], scale)


def _save_video(frames: list[np.ndarray], path: Path, fps: int) -> None:
    import cv2

    if not frames:
        return
    height, width = frames[0].shape[:2]
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*'mp4v'),
        float(fps),
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f'Failed to open video writer: {path}')
    for frame in frames:
        writer.write(frame)
    writer.release()


def _run_trial(
    env,
    *,
    spawn: tuple[float, float, float],
    target: list[float],
    max_steps: int,
    max_vx: float,
    max_vy: float,
    max_w: float,
    pos_tol: float,
    yaw_tol: float,
    speed_tol: float,
    record_video: bool,
    video_source: str,
    video_scale: int,
) -> tuple[bool, int, list[np.ndarray]]:
    from autonomy_navrl.control.precision_controller import (
        compute_pose_tracking_action,
        is_fully_aligned,
    )

    env.reset()
    sx, sy, syaw = spawn
    env.set_robot_spawn(sx, sy, syaw)
    obs = env.observation_dict()
    frames: list[np.ndarray] = []
    if record_video:
        frame = _capture_frame(env, obs, source=video_source, scale=video_scale)
        if frame is not None:
            frames.append(frame)

    success = False
    final_step = max_steps - 1
    for step in range(max_steps):
        state = obs['state']
        dx_body, dy_body, distance, bearing, yaw_error, speed = _parse_state(state[0])
        action = compute_pose_tracking_action(
            dx_body, dy_body, distance, bearing, yaw_error,
            max_vx=max_vx, max_vy=max_vy, max_w=max_w,
        )
        result = env.step(np.tile(action, (env.num_envs, 1)))
        obs = result.observation
        if record_video:
            frame = _capture_frame(env, obs, source=video_source, scale=video_scale)
            if frame is not None:
                frames.append(frame)

        if step % 50 == 0 or result.terminated.any() or result.truncated.any():
            print(
                f'[batch] spawn=({sx:.2f},{sy:.2f},{math.degrees(syaw):+.0f}°) '
                f'step={step:4d} dist={distance:.3f}m '
                f'yaw_err={math.degrees(yaw_error):+.1f}°',
                flush=True,
            )

        aligned = is_fully_aligned(
            distance, yaw_error, speed,
            pos_tol_m=pos_tol, yaw_tol_rad=yaw_tol, speed_tol_mps=speed_tol,
        )
        if result.terminated.any() or aligned:
            success = bool(aligned or result.terminated.any())
            final_step = step
            break
        if result.truncated.any():
            final_step = step
            break
    return success, final_step, frames


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
        if args.livestream > 0:
            config.setdefault('viz', {})['enabled'] = True
            config['viz'].setdefault('auto_enable_on_livestream', True)

    record_video = not args.no_video
    if record_video:
        video_scale = args.video_scale
        if args.video_source == 'onboard' and video_scale == 1:
            video_scale = 4
        config.setdefault('video', {})['source'] = args.video_source
        config.setdefault('isaac', {})['enable_cameras'] = True
    else:
        video_scale = args.video_scale

    task_cfg = config.setdefault('task', {})
    target = list(task_cfg.get('target_pose', [2.0, 0.0, 0.0]))
    if args.target_x is not None:
        target[0] = args.target_x
    if args.target_y is not None:
        target[1] = args.target_y
    if args.target_yaw_deg is not None:
        target[2] = math.radians(args.target_yaw_deg)
    task_cfg['target_pose'] = target
    task_cfg.pop('pose_range', None)

    control_cfg = config.get('control', {})
    max_vx = float(control_cfg.get('max_vx', 0.5))
    max_vy = float(control_cfg.get('max_vy', 0.5))
    max_w = float(control_cfg.get('max_w', 1.0))
    pos_tol = float(task_cfg.get('position_tolerance_m', task_cfg.get('goal_tolerance_m', 0.05)))
    yaw_tol = float(task_cfg.get('yaw_tolerance_rad', 0.1))
    speed_tol = float(task_cfg.get('require_stop_speed_mps', 0.08))

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    if args.video_dir:
        video_dir = Path(args.video_dir)
    else:
        video_dir = Path('checkpoints') / 'go2w_batch_demo' / 'videos' / stamp
    video_dir.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(args.seed)
    spawn_samples = []
    for _ in range(args.trials):
        spawn_samples.append((
            float(rng.uniform(args.spawn_x_min, args.spawn_x_max)),
            float(rng.uniform(args.spawn_y_min, args.spawn_y_max)),
            float(math.radians(rng.uniform(args.spawn_yaw_min_deg, args.spawn_yaw_max_deg))),
        ))

    print(
        f'[batch] trials={args.trials} target=({target[0]:.2f},{target[1]:.2f},'
        f'{math.degrees(target[2]):.0f}°) video_dir={video_dir.resolve()} '
        f'video_source={args.video_source}',
        flush=True,
    )

    sim_app = bootstrap_isaac_app(config)
    env = create_training_env(config, backend='isaac')
    results: list[dict] = []
    try:
        for index, spawn in enumerate(spawn_samples, start=1):
            print(
                f'\n[batch] === trial {index}/{args.trials} '
                f'spawn x={spawn[0]:.2f} y={spawn[1]:.2f} yaw={math.degrees(spawn[2]):+.1f}° ===',
                flush=True,
            )
            success, steps, frames = _run_trial(
                env,
                spawn=spawn,
                target=target,
                max_steps=args.max_steps,
                max_vx=max_vx,
                max_vy=max_vy,
                max_w=max_w,
                pos_tol=pos_tol,
                yaw_tol=yaw_tol,
                speed_tol=speed_tol,
                record_video=record_video,
                video_source=args.video_source,
                video_scale=video_scale,
            )
            video_path = video_dir / f'trial_{index:02d}.mp4'
            if record_video:
                _save_video(frames, video_path, args.video_fps)
                print(f'[batch] video saved: {video_path.resolve()}', flush=True)
            row = {
                'trial': index,
                'spawn_x': spawn[0],
                'spawn_y': spawn[1],
                'spawn_yaw_deg': math.degrees(spawn[2]),
                'success': success,
                'steps': steps,
                'video_source': args.video_source,
                'video': str(video_path) if record_video else '',
            }
            results.append(row)
            print(
                f'[batch] trial {index} {"SUCCESS" if success else "FAIL"} steps={steps}',
                flush=True,
            )

        summary_path = video_dir / 'summary.json'
        summary_path.write_text(json.dumps(results, indent=2), encoding='utf-8')
        success_count = sum(1 for row in results if row['success'])
        print(
            f'\n[batch] done: {success_count}/{args.trials} success, '
            f'summary={summary_path.resolve()}',
            flush=True,
        )
        return 0 if success_count == args.trials else 1
    finally:
        env.close()
        sim_app.close()


if __name__ == '__main__':
    sys.exit(main())
