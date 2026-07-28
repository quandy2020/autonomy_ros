"""Bag episode 适配层：把 LeRobot v3 目录结构 + video 解码成 Sample 迭代器。
只 import 现有数据接口（process_data_parquet / load_depth），不动 NavDP_Base_Datset 源文件。
"""
from __future__ import annotations
from collections import namedtuple
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List

import imageio.v2 as imageio
import numpy as np
import pandas as pd
import sys

# 数据契约：单帧全部入参
Sample = namedtuple('Sample', [
    'frame_id',          # int
    'rgb',               # (H,W,3) uint8 RGB
    'depth',             # (H,W) float32 米
    'intrinsic',         # (3,3)
    'extrinsic',         # (4,4) cam->world，当前帧
    'gt_traj_world',     # (T,3) 未来 T 帧世界系 (x,y,yaw) 累计
])


@dataclass
class Episode:
    """一个 episode 的静态元数据 + 解码器。"""
    episode_id: int
    rgb_paths: List[str]        # mp4 视频路径（按帧索引）
    depth_paths: List[str]      # depth 视频路径
    intrinsic: np.ndarray       # (3,3)
    extrinsics: np.ndarray      # (N,4,4) 全帧 GT cam->world
    camera_traj_world: np.ndarray  # (N,4,4) 全帧 GT base->world
    episode_slice: tuple = (0, 0)   # (start, end) 该 ep 在全局帧序列中的范围

    @property
    def n_frames(self) -> int:
        return self.episode_slice[1] - self.episode_slice[0]


class BagEpisodeLoader:
    """扫描 bag_root，按 scene 聚合 episode，提供 iter_episodes()。"""

    def __init__(self, bag_root: str, horizon: int):
        self.bag_root = Path(bag_root)
        self.horizon = horizon
        if not self.bag_root.exists():
            raise FileNotFoundError(f'bag_root 不存在: {self.bag_root}')

    def iter_episodes(self) -> Iterator[Episode]:
        ep_id = 0
        # 单层 LeRobot v3（bag_root 本身就是 scene：含 data/videos/meta）
        if (self.bag_root / 'data').exists() and (self.bag_root / 'videos').exists():
            yield from self._iter_scene_episodes(self.bag_root, ep_id)
            return
        # 平铺：bag_root 下每个 run_X 子目录是一个 scene
        for scene_dir in sorted(self.bag_root.iterdir()):
            if not scene_dir.is_dir():
                continue
            if not (scene_dir / 'data').exists():
                continue
            try:
                for ep in self._iter_scene_episodes(scene_dir, ep_id):
                    yield ep
                    ep_id += 1
            except FileNotFoundError:
                continue
        # 双层 LeRobot v3（<group>/<scene>）兼容
        for group_dir in sorted(self.bag_root.iterdir()):
            if not group_dir.is_dir():
                continue
            if (group_dir / 'data').exists():
                continue  # 已被平铺分支处理
            for scene_dir in sorted(group_dir.iterdir()):
                if not scene_dir.is_dir() or not (scene_dir / 'data').exists():
                    continue
                try:
                    for ep in self._iter_scene_episodes(scene_dir, ep_id):
                        yield ep
                        ep_id += 1
                except FileNotFoundError:
                    continue

    def _iter_scene_episodes(self, scene_dir: Path, start_id: int) -> Iterator[Episode]:
        meta_dir = scene_dir / 'meta'
        data_root = scene_dir / 'data'
        videos_root = scene_dir / 'videos'
        if not (meta_dir.exists() and data_root.exists() and videos_root.exists()):
            print(f"[loader] skip {scene_dir}: meta/data/videos 缺失", file=sys.stderr)
            return

        chunks = sorted(p.name for p in data_root.iterdir() if p.is_dir())
        if not chunks:
            return
        chunk_name = chunks[0]
        data_dir = data_root / chunk_name
        # 兼容两种布局：
        #  A) videos/<modality>/<chunk>/file-XXX.mp4  (单层 bag)
        #  B) videos/<chunk>/<modality>/file-XXX.mp4  (标准 LeRobot v3)
        rgb_dir = videos_root / 'observation.images.rgb' / chunk_name
        depth_dir = videos_root / 'observation.images.depth' / chunk_name
        if not (rgb_dir.exists() and depth_dir.exists()):
            rgb_dir = videos_root / chunk_name / 'observation.images.rgb'
            depth_dir = videos_root / chunk_name / 'observation.images.depth'
        if not (rgb_dir.exists() and depth_dir.exists()):
            return
        rgb_videos = sorted(rgb_dir.iterdir())
        depth_videos = sorted(depth_dir.iterdir())
        data_parquets = sorted(p for p in data_dir.iterdir() if p.suffix == '.parquet')
        if not (rgb_videos and depth_videos and data_parquets):
            return

        # episodes 索引：优先 episodes_stats.jsonl，否则按 parquet 文件切
        stats_path = meta_dir / 'episodes_stats.jsonl'
        if stats_path.exists():
            episodes_meta = pd.read_json(stats_path, lines=True).to_dict('records')
        else:
            # 把每个 parquet file 当作 1 个 episode（bag 录制连续）
            episodes_meta = [{'image_index': {'min': 0, 'max': None}}] * len(data_parquets)

        for ep_idx, ep_meta in enumerate(episodes_meta):
            if ep_idx >= len(data_parquets):
                break
            intrinsic, extrinsics, camera_traj, n_rows = _read_parquet(data_parquets[ep_idx])
            i0 = ep_meta['image_index']['min']
            i1 = ep_meta['image_index'].get('max') if isinstance(ep_meta['image_index'], dict) else None
            if i1 is None:
                i1 = i0 + n_rows - 1
            yield Episode(
                episode_id=start_id + ep_idx,
                episode_slice=(i0, i1 + 1),
                rgb_paths=[str(p) for p in [rgb_videos[0]]],
                depth_paths=[str(p) for p in [depth_videos[0]]],
                intrinsic=intrinsic,
                extrinsics=extrinsics,
                camera_traj_world=camera_traj,
            )


def _read_parquet(path: Path):
    """解码 LeRobot v3 字段。
    intrinsic: 全行共享 (3,3)
    extrinsics: (N,4,4) cam→base
    camera_traj_world: (N,4,4) base→world，优先取 action；缺失则用 cam_extrinsic 逆矩阵
    """
    df = pd.read_parquet(path)
    intrinsic_flat = np.array(df['observation.camera_intrinsic'].tolist()[0], dtype=np.float64).reshape(3, 3)
    extrinsics = np.array([np.asarray(x, dtype=np.float64).reshape(4, 4) for x in df['observation.camera_extrinsic'].tolist()])
    if 'action' in df.columns:
        camera_traj = np.array([np.asarray(a, dtype=np.float64).reshape(4, 4) for a in df['action'].tolist()])
    else:
        # fallback：base→world ≈ cam→world，假设 cam→base=I（楼梯数据集静态外参）
        camera_traj = np.tile(np.eye(4, dtype=np.float64), (len(df), 1, 1))
    return (intrinsic_flat.astype(np.float32),
            extrinsics.astype(np.float32),
            camera_traj.astype(np.float32),
            len(df))


def iter_samples(ep: Episode, horizon: int) -> Iterator[Sample]:
    """逐帧解码 mp4 → Sample，直到 T-horizon 不够为止。
    extrinsic: cam→world = extrinsics[t] @ camera_traj_world[t]
    gt_traj_world: 未来 horizon 帧的 base 世界平移 (x,y,z)
    """
    rgb_reader = imageio.get_reader(ep.rgb_paths[0])
    depth_reader = imageio.get_reader(ep.depth_paths[0])
    try:
        i0, i1 = ep.episode_slice
        n = min(rgb_reader.count_frames(), ep.extrinsics.shape[0], i1)
        for t in range(i0, n):
            rgb = rgb_reader.get_data(t)              # (H,W,3) uint8 RGB
            depth_raw = depth_reader.get_data(t)      # (H,W[,3]) uint16
            # depth mp4 偶尔被容器编成 3 通道，取第 0 通道
            if depth_raw.ndim == 3:
                depth_raw = depth_raw[..., 0]
            depth_m = depth_raw.astype(np.float32) / 10000.0
            # cam→base @ base→world = cam→world
            cam_to_world = ep.extrinsics[t] @ ep.camera_traj_world[t]
            # GT 未来 horizon 帧的世界系 (x,y,z)（取前 3 维，yaw 丢弃）
            future = ep.camera_traj_world[t:t + horizon, :3, 3]  # (T,3) 世界平移
            yield Sample(
                frame_id=t,
                rgb=rgb,
                depth=depth_m,
                intrinsic=ep.intrinsic,
                extrinsic=cam_to_world,
                gt_traj_world=future,
            )
    finally:
        rgb_reader.close()
        depth_reader.close()