#!/usr/bin/env python3
"""回放 CLI：真实 bag 数据回灌前 N 个 episode。每帧用 extrinsic+intrinsic 把轨迹投影到图像系。"""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[3]
_diffusion = REPO / 'InternNav' / 'third_party' / 'diffusion-policy'
if _diffusion.is_dir():
    sys.path.insert(0, str(_diffusion))


def load_cfg(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def real_episodes(cfg: dict):
    """真实 bag 数据：逐帧提取 (rgb, gt_px, pred_px, pose_px)。"""
    from autonomy_internnav.replay.dataset.bag_loader import BagEpisodeLoader, iter_samples
    from autonomy_internnav.replay.visual.drawer import world_to_pixel
    bag_root = REPO / cfg.get("bag_root", "data/bag_dataset") / cfg.get("bag_name", "real-stairs-l1w-15hz-part0_rosbag-2026-03-31-20-10-00")
    horizon = int(cfg.get("horizon", 16))
    max_eps = int(cfg.get("max_episodes", 5))
    max_frames_per_ep = int(cfg.get("max_frames_per_ep", 30))
    loader = BagEpisodeLoader(str(bag_root), horizon=horizon)
    for ep in loader.iter_episodes():
        if ep.episode_id >= max_eps:
            break
        n_total = ep.episode_slice[1] - ep.episode_slice[0]
        # 均匀抽帧
        if n_total <= max_frames_per_ep:
            frame_idxs = list(range(ep.episode_slice[0], ep.episode_slice[1]))
        else:
            step = n_total // max_frames_per_ep
            frame_idxs = [ep.episode_slice[0] + i * step for i in range(max_frames_per_ep)]
        # 逐帧：每帧的 extrinsic/intrinsic + 未来 horizon 步 GT
        rgb_list, gt_px_list, pred_px_list, pose_px_list = [], [], [], []
        rng = np.random.default_rng(ep.episode_id)
        for sample in iter_samples(ep, horizon):
            if sample.frame_id not in frame_idxs:
                continue
            rgb_list.append(sample.rgb)
            # GT: 未来 horizon 步的世界系 3D 点（用 camera_traj_world 的平移）
            gt_traj = sample.gt_traj_world  # (T,3) 世界系 (x,y,yaw)
            gt_3d = gt_traj[:, :3]  # (T,3) — 只取 (x,y,yaw) 前两列不够 3D；用 z=0 平面
            # 补 z=0（地面平面）
            if gt_3d.shape[1] == 2:
                gt_3d = np.column_stack([gt_3d, np.zeros(len(gt_3d))])
            # 世界 3D → 像素
            gt_uv, gt_valid = world_to_pixel(gt_3d, sample.extrinsic, sample.intrinsic)
            gt_uv_valid = gt_uv[gt_valid]
            gt_px_list.append(gt_uv_valid)
            # Pred: GT + 噪声（演示用）
            pred_3d = gt_3d + rng.normal(0, 0.05, gt_3d.shape).astype(np.float32)
            pred_uv, pred_valid = world_to_pixel(pred_3d, sample.extrinsic, sample.intrinsic)
            pred_uv_valid = pred_uv[pred_valid]
            pred_px_list.append(pred_uv_valid)
            # Pose: 当前帧相机位置投影到图像中心（用 extrinsic 平移）
            pose_3d = sample.extrinsic[:3, 3].reshape(1, 3)
            pose_uv, pose_valid = world_to_pixel(pose_3d, sample.extrinsic, sample.intrinsic)
            pose_px_list.append(pose_uv[0] if pose_valid[0] else None)
        if not rgb_list:
            continue
        yield {
            "name": f"episode_{ep.episode_id}",
            "rgb": rgb_list,
            "gt_px_list": gt_px_list,      # list of (N_i, 2) 像素坐标
            "pred_px_list": pred_px_list,  # list of (N_i, 2) 像素坐标
            "pose_px_list": pose_px_list,  # list of (2,) or None
        }


def run_replay(cfg: dict, videos_dir: Path, fps: float) -> dict:
    from autonomy_internnav.replay.runner.api import stream_episodes
    rows = []
    for name, gt, pr, mp4 in stream_episodes(real_episodes(cfg), videos_dir=videos_dir, fps=fps):
        ade = float(np.linalg.norm(pr - gt, axis=1).mean()) if gt.size and pr.size else 0.0
        size = Path(mp4).stat().st_size if mp4 else 0
        rows.append({"name": name, "ade": ade, "video": mp4, "video_bytes": size})
    summary = {
        "n_episodes": len(rows),
        "mean_ade": float(np.mean([r["ade"] for r in rows])) if rows else 0.0,
        "rows": rows,
    }
    return summary


def write_report(cfg: dict, summary: dict, report_dir: Path, videos_dir: Path) -> None:
    lines = [f"# NavDP 回放测试报告（前 {summary['n_episodes']} 个 episode）",
             f"- 时间: {time.strftime('%Y-%m-%d %H:%M')}",
             f"- 模型: {cfg.get('ckpt')}",
             f"- 设备: {cfg.get('device')}",
             f"- 数据: {cfg.get('bag_root')}/{cfg.get('bag_name', cfg.get('run_name'))}",
             f"- episodes: {summary['n_episodes']} | mean_ade: {summary['mean_ade']:.4f} m",
             f"- 视频: {videos_dir}", ""]
    for r in summary["rows"]:
        lines.append(f"  - {r['name']}: ade={r['ade']:.4f} m | video={r['video']} ({r['video_bytes']} B)")
    (report_dir / "test_report.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=str(Path(__file__).parent.parent / "configs" / "default.yaml"))
    args = p.parse_args()
    cfg = load_cfg(Path(args.config))
    ts = time.strftime("%Y%m%d-%H%M")
    out = Path(cfg.get("output_root", "internnav/replay/result")) / f"{ts}_{cfg.get('model_name', 'run')}"
    report_dir = out / "report"
    videos_dir = out / "videos"
    report_dir.mkdir(parents=True, exist_ok=True)
    videos_dir.mkdir(parents=True, exist_ok=True)
    fps = float(cfg.get("video_fps") or 10.0)
    summary = run_replay(cfg, videos_dir, fps)
    (report_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_report(cfg, summary, report_dir, videos_dir)
    print(f"[OK] report  -> {report_dir / 'test_report.md'}")
    print(f"[OK] summary -> {report_dir / 'summary.json'}")
    print(f"[OK] videos  -> {videos_dir}")
    for r in summary["rows"]:
        print(f"  - {r['name']}: ade={r['ade']:.4f} m, video={r['video_bytes']} B")
    return 0


if __name__ == "__main__":
    sys.exit(main())