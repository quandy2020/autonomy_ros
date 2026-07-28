# Adapter that exposes a LeRobotDataset v3 directory tree to the NavDP training
# pipeline without rewriting any augmentation logic.
#
# Design contract
# ---------------
#   * __init__ skips the scene/parquet directory walk of NavDP_Base_Datset and
#     instead holds one (or many) LeRobotDataset instances. Per-episode global
#     frame ranges are pre-computed from `meta.episodes[*]['dataset_from_index']`
#     / `dataset_to_index`.
#   * `trajectory_*_path` lists are populated with opaque keys instead of file
#     paths, so that the parent class can keep indexing them transparently:
#         - trajectory_data_dir[i]   -> ('parquet', dataset_idx, ep_idx)
#         - trajectory_rgb_path[i]   -> [('rgb',   dataset_idx, ep_idx, t), ...]
#         - trajectory_depth_path[i] -> [('depth', dataset_idx, ep_idx, t), ...]
#         - trajectory_afford_path[i]-> '<scene_root>/meta/pointcloud.ply'
#   * Five I/O hooks are overridden: `__init__`, `load_image`, `load_depth`,
#     `process_data_parquet`, `process_obstacle_points`. `process_pixel_goal`
#     is also overridden, but only to reroute its raw `Image.open` call through
#     `self.load_image`; its projection / mask / pad / resize math is copied
#     verbatim from the parent. All other geometry / augmentation methods
#     (process_image, process_depth, process_memory, process_actions,
#     xyz_to_xyt, relative_pose, absolute_pose, rank_steps, __getitem__) are
#     inherited unchanged.
#
# Depth contract
# --------------
# convert_to_navdp.py writes depth video as (depth_m / MAX_DEPTH_M) * 255 over
# three identical channels. After torchvision decode each pixel sits in [0, 1]
# (float32). To preserve the parent's `load_depth() / 10000.0 -> meters` math we
# multiply by `MAX_DEPTH_M * 10000` here so that the inherited `process_depth`
# treats us exactly like a real uint16 millimeter PNG.

import json
import os
from collections import OrderedDict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import pandas as pd
import torch

from autonomy_internnav.train.dataset import NavDP_Base_Datset

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError as exc:  # pragma: no cover - runtime guard
    raise ImportError(
        "lerobot>=0.4 is required for NavDP_LerobotV3_Dataset; "
        "please `pip install lerobot`."
    ) from exc


# Must match `max_depth_m` used in convert_to_navdp.py when normalising depth
# video frames. Keep in sync if the converter ever changes that constant.
MAX_DEPTH_M = 5.0


def _decode_frames_pyav(video_path, timestamps, tolerance_s):
    """Decode video frames closest to *timestamps* using PyAV.

    Uses frame-index seek for efficiency: decode only the needed segment
    around each target timestamp, not the entire video.

    Returns a single (C,H,W) tensor if one timestamp is given,
    or a stacked (N,C,H,W) tensor for multiple timestamps.
    """
    import av

    container = av.open(video_path)
    stream = container.streams.video[0]
    stream.thread_type = "AUTO"
    time_base = float(stream.time_base)

    # Sort timestamps to enable sequential decode
    sorted_ts = sorted(enumerate(timestamps), key=lambda x: x[1])

    # Seek to just before the earliest timestamp and decode forward
    if sorted_ts:
        first_ts = sorted_ts[0][1]
        seek_target = max(0, first_ts - tolerance_s)
        container.seek(int(seek_target / time_base), stream=stream)

    # Build pts→frame mapping by decoding forward through the video
    frames_by_ts = {}
    cur_decoded_pts = []
    for pkt in container.demux(stream):
        for f in pkt.decode():
            f_ts = f.pts * time_base
            cur_decoded_pts.append((f_ts, f))
            # Check if this frame satisfies any pending timestamp
            for orig_idx, target_ts in sorted_ts:
                if abs(f_ts - target_ts) <= tolerance_s and orig_idx not in frames_by_ts:
                    frames_by_ts[orig_idx] = f
            # Early exit: if all timestamps past the last target, stop
            if cur_decoded_pts and len(frames_by_ts) == len(timestamps):
                if f_ts > sorted_ts[-1][1] + tolerance_s:
                    break
        if len(frames_by_ts) == len(timestamps):
            break
    container.close()

    h, w = stream.height, stream.width
    result = [None] * len(timestamps)
    for orig_idx, target_ts in sorted_ts:
        f = frames_by_ts.get(orig_idx)
        if f is not None:
            img = f.to_ndarray(format="rgb24")
            result[orig_idx] = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        else:
            result[orig_idx] = torch.zeros(3, h, w)

    if len(result) == 1:
        return result[0]
    return torch.stack(result)


# How deep we descend when auto-discovering LeRobotDataset roots inside a parent
# directory. Each match is the parent of a `meta/info.json`. A modest cap keeps
# discovery cheap on large data trees.
_AUTO_DISCOVER_MAX_DEPTH = 6

# Memory budget (in bytes) for `_frame_cache`. Each decoded frame is a
# native-resolution (C, H, W) float32 array — e.g. 3x480x640 ~= 3.5 MiB — so a
# count-based bound is dangerously misleading: 4096 such frames would be ~14 GiB.
# We therefore evict by *bytes*, not by entry count. GRPO samples a small set of
# scenes per round, so a few hundred MiB comfortably holds the hot working set.
# Override via the JDINTERNNAV_FRAME_CACHE_MB env var if needed.
_FRAME_CACHE_MAX_BYTES = int(os.environ.get("JDINTERNNAV_FRAME_CACHE_MB", "512")) * 1024 * 1024

# Memory budget (in bytes) for `_pointcloud_cache`. Each entry holds the masked
# obstacle + inflation point arrays for one scene ([N, 3] float32 each). Scene
# selection rotates over a long run, so without a bound every scene ever touched
# would accumulate. LRU by bytes keeps the recently sampled scenes only.
_POINTCLOUD_CACHE_MAX_BYTES = int(os.environ.get("JDINTERNNAV_PCD_CACHE_MB", "512")) * 1024 * 1024


def _candidate_repo_roots():
    """Return absolute base paths used to resolve relative root_dir values.

    Order matters: the first existing path wins. Falls back to CWD so behaviour
    on absolute paths is unchanged.
    """
    here = Path(__file__).resolve()
    # internnav/dataset/<file> -> repo is two levels up from `internnav`.
    repo_root = here.parents[2]
    return [
        Path(os.environ.get("JDINTERNNAV_ROOT", "")) if os.environ.get("JDINTERNNAV_ROOT") else None,
        repo_root,
        Path.cwd(),
    ]


def _resolve_root(path_like) -> Path:
    """Make `path_like` absolute by trying known repo roots, then CWD."""
    p = Path(path_like)
    if p.is_absolute():
        return p.resolve()
    for base in _candidate_repo_roots():
        if base is None:
            continue
        candidate = (base / p).resolve()
        if candidate.exists():
            return candidate
    # Last resort: resolve against CWD even if it does not exist, so the error
    return p.resolve()


def _resolve_pointcloud_path(root: Path, pointcloud_name: str) -> str:
    """Resolve scene obstacle pointcloud for a LeRobot v3 dataset root."""
    candidates = (
        root / 'meta' / pointcloud_name,
        root / pointcloud_name,
        root.parent / pointcloud_name,
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate.resolve())
    return str((root / 'meta' / pointcloud_name).resolve())


def _discover_lerobot_roots(root: Path) -> list[Path]:
    """Return all LeRobotDataset root directories found under `root`.

    A root is any directory that directly contains `meta/info.json`. If `root`
    itself qualifies, it is returned alone (single-dataset mode). Otherwise a
    bounded recursive scan returns every nested match (multi-dataset mode).
    """
    if (root / "meta" / "info.json").is_file():
        return [root]

    found: list[Path] = []
    root_depth = len(root.parts)
    for cur, dirs, files in os.walk(root, followlinks=False):
        cur_path = Path(cur)
        # Stop descending when we go past the configured depth budget.
        if len(cur_path.parts) - root_depth > _AUTO_DISCOVER_MAX_DEPTH:
            dirs[:] = []
            continue
        if cur_path.name == "meta" and "info.json" in files:
            found.append(cur_path.parent)
            # No reason to descend further once we already located a root.
            dirs[:] = []
    return sorted(found)


def _ensure_tasks_parquet(root: str) -> bool:
    """Create meta/tasks.parquet if missing so LeRobotDataset.load_metadata won't
    raise FileNotFoundError and fall back to huggingface_hub (which fails under
    HF_HUB_OFFLINE=1). Some converted datasets ship without this file.

    Returns True if tasks.parquet exists (pre-existing or rebuilt).
    """
    tasks_path = os.path.join(root, "meta", "tasks.parquet")
    if os.path.isfile(tasks_path):
        return True
    info_path = os.path.join(root, "meta", "info.json")
    task_name = "navigate"
    total_tasks = 1
    if os.path.isfile(info_path):
        try:
            with open(info_path) as f:
                info = json.load(f)
            total_tasks = info.get("total_tasks", 1)
            task_name = info.get("task", task_name)
        except Exception:
            pass
    tasks_df = pd.DataFrame(
        {"task_index": list(range(total_tasks))},
        index=[f"{task_name}_{i}" for i in range(total_tasks)],
    )
    tasks_df.to_parquet(tasks_path)
    return True


def _scan_video_keys(root: str) -> list[str]:
    """Return video feature keys by scanning videos/ subdirectories."""
    vids_dir = os.path.join(root, "videos")
    if not os.path.isdir(vids_dir):
        return []
    return sorted(
        d for d in os.listdir(vids_dir)
        if os.path.isdir(os.path.join(vids_dir, d))
    )


def _scan_video_chunks(root: str, vid_key: str) -> dict:
    """Return {chunk_index: file_index} from videos/<vid_key>/ directory."""
    vid_dir = os.path.join(root, "videos", vid_key)
    if not os.path.isdir(vid_dir):
        return {0: 0}
    chunks = {}
    for chunk_name in sorted(os.listdir(vid_dir)):
        if not chunk_name.startswith("chunk-"):
            continue
        try:
            cidx = int(chunk_name.split("-")[1])
        except (ValueError, IndexError):
            continue
        chunk_dir = os.path.join(vid_dir, chunk_name)
        for fname in sorted(os.listdir(chunk_dir)):
            if fname.endswith(".mp4"):
                try:
                    fidx = int(fname.replace("file-", "").replace(".mp4", ""))
                    chunks[cidx] = fidx
                except ValueError:
                    pass
    return chunks if chunks else {0: 0}


def _ensure_episodes_parquet(root: str) -> bool:
    """Create meta/episodes/ parquet if missing.

    Rebuilds episode metadata from data parquet and video directory structure.
    load_episodes() filters out stats/ columns, so we omit them entirely.

    Returns True if episodes parquet exists (pre-existing or rebuilt).
    Returns False if rebuild fails (missing data or zero episodes).
    """
    ep_dir = os.path.join(root, "meta", "episodes")
    if os.path.isdir(ep_dir) and any(
        os.path.isfile(os.path.join(dp, f))
        for dp, _, fns in os.walk(ep_dir)
        for f in fns
        if f.endswith(".parquet")
    ):
        return True

    info_path = os.path.join(root, "meta", "info.json")
    if not os.path.isfile(info_path):
        return False
    with open(info_path) as f:
        info = json.load(f)

    fps = info.get("fps", 20)
    total_episodes = info.get("total_episodes", 0)
    if total_episodes <= 0:
        return False

    # Load episode boundaries from data parquet
    data_dir = os.path.join(root, "data")
    data_files = sorted(
        os.path.join(dp, fn)
        for dp, _, fns in os.walk(data_dir)
        for fn in fns
        if fn.endswith(".parquet")
    )
    if not data_files:
        return False

    import pyarrow.parquet as pq
    # Merge episode groups across data files, combining counts for the same
    # episode_index (each data chunk may cover a disjoint subset of frames).
    ep_length_map: dict[int, int] = {}
    for fpath in data_files:
        tbl = pq.read_table(fpath, columns=["episode_index"])
        df = tbl.to_pandas()
        for ep_idx in df["episode_index"].unique():
            ep_idx = int(ep_idx)
            n = int((df["episode_index"] == ep_idx).sum())
            ep_length_map[ep_idx] = ep_length_map.get(ep_idx, 0) + n
    ep_groups = sorted(ep_length_map.items())

    task_name = info.get("task", "navigate")
    vid_keys = _scan_video_keys(root)

    rows = []
    offset = 0
    for ep_idx, length in ep_groups:
        from_idx = offset
        to_idx = offset + length
        row = {
            "episode_index": ep_idx,
            "tasks": [f"{task_name}_{0}"],
            "length": length,
            "data/chunk_index": 0,
            "data/file_index": 0,
            "dataset_from_index": from_idx,
            "dataset_to_index": to_idx,
            "meta/episodes/chunk_index": 0,
            "meta/episodes/file_index": 0,
        }
        for vk in vid_keys:
            chunks = _scan_video_chunks(root, vk)
            cidx = min(chunks.keys())
            row[f"videos/{vk}/chunk_index"] = cidx
            row[f"videos/{vk}/file_index"] = chunks[cidx]
            row[f"videos/{vk}/from_timestamp"] = from_idx / fps
            row[f"videos/{vk}/to_timestamp"] = to_idx / fps
        rows.append(row)
        offset = to_idx

    ep_df = pd.DataFrame(rows)
    out_path = os.path.join(ep_dir, "chunk-000", "file-000.parquet")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    ep_df.to_parquet(out_path)
    return True


class NavDP_LerobotV3_Dataset(NavDP_Base_Datset):
    """NavDP dataset adapter that consumes one or more LeRobotDataset v3 roots.

    Args:
        root_dirs: A single dataset root path (containing `meta/`, `data/`,
            `videos/`) or a list of such paths. Each root is wrapped in its own
            LeRobotDataset instance; episodes are flattened across them.
        rgb_key / depth_key: Names of the LeRobot video features.
        pointcloud_name: Relative path (under each root) of the obstacle
            pointcloud. Defaults to `meta/pointcloud.ply` per NavDP convention.
        dataset_repeat: Multiplier applied to the flattened episode list, mirrors
            the `* 50` trick used by NavDP_Base_Datset.
        Remaining kwargs are forwarded to the parent class' hyper-parameters.
    """

    def __init__(
        self,
        root_dirs,
        preload_path=False,  # kept for API parity, unused on v3
        memory_size=8,
        predict_size=24,
        batch_size=64,
        image_size=224,
        scene_data_scale=1.0,
        trajectory_data_scale=1.0,
        pixel_channel=7,
        action_dim=3,
        debug=False,
        preload=False,  # kept for API parity, unused on v3
        random_digit=False,
        prior_sample=False,
        dataset_repeat=1,
        rgb_key="observation.images.rgb",
        depth_key="observation.images.depth",
        pointcloud_name="pointcloud.ply",
        target_segment_weights=None,
        sample_interval=4,
        use_data_filter=True,
        max_continuous_step_m=0.12,
        max_continuous_yaw_rad=0.70,
        max_filter_sample_tries=64,
        use_pointcloud=True,
    ):
        # Intentionally skip NavDP_Base_Datset.__init__ to avoid its directory
        # walk; replicate only the attribute initialisation we depend on.
        torch.utils.data.Dataset.__init__(self)

        self.memory_size = memory_size
        self.image_size = image_size
        self.scene_scale_size = scene_data_scale
        self.trajectory_data_scale = trajectory_data_scale
        self.predict_size = predict_size
        self.action_dim = action_dim
        self.debug = debug
        self.random_digit = random_digit
        self.prior_sample = prior_sample
        self.sample_interval = max(1, int(sample_interval))
        self.use_data_filter = bool(use_data_filter)
        self.max_continuous_step_m = float(max_continuous_step_m)
        self.max_continuous_yaw_rad = float(max_continuous_yaw_rad)
        self.max_filter_sample_tries = max(1, int(max_filter_sample_tries))
        self.pixel_channel = pixel_channel
        self.batch_size = batch_size
        self.item_cnt = 0
        self.batch_time_sum = 0.0
        self._last_time = None
        self.dataset_repeat = max(1, int(dataset_repeat))

        self.rgb_key = rgb_key
        self.depth_key = depth_key
        self.target_segment_weights = target_segment_weights  # 目标帧分段权重, None 表示均匀随机
        self.obstacle_sample_n = 2048  # 障碍点采样数量
        self.use_pointcloud = bool(use_pointcloud)

        # Normalise input into a list of LeRobotDataset roots, resolving any
        # relative paths against the repository / CWD so the launcher's working
        # directory does not matter. For each entry we either treat it as a
        # single LeRobotDataset root (contains `meta/info.json` directly) or
        # auto-discover every nested LeRobotDataset root underneath it.
        if isinstance(root_dirs, (str, bytes, os.PathLike)):
            input_paths = [root_dirs]
        else:
            input_paths = list(root_dirs)

        resolved_roots: list[Path] = []
        for raw in input_paths:
            base = _resolve_root(raw)
            if not base.exists():
                raise FileNotFoundError(
                    f"NavDP_LerobotV3_Dataset: root path does not exist: "
                    f"{raw!r} (resolved to {base})"
                )
            discovered = _discover_lerobot_roots(base)
            if not discovered:
                raise FileNotFoundError(
                    f"NavDP_LerobotV3_Dataset: no LeRobotDataset root found "
                    f"under {base} (looked for `**/meta/info.json` up to "
                    f"depth {_AUTO_DISCOVER_MAX_DEPTH})"
                )
            resolved_roots.extend(discovered)

        # De-duplicate while preserving discovery order.
        seen: set[str] = set()
        root_list: list[str] = []
        for r in resolved_roots:
            key = str(r)
            if key not in seen:
                seen.add(key)
                root_list.append(key)

        self._datasets: list[LeRobotDataset] = []
        # Each entry: (dataset_idx, ep_idx, ep_from, ep_to, pointcloud_path)
        episode_records: list[tuple[int, int, int, int, str]] = []

        import logging
        _logger = logging.getLogger(__name__)

        skipped_roots: list[str] = []
        for ds_idx, root in enumerate(root_list):
            # Pre-validate to keep error messages local instead of bubbling
            # up as a misleading 401 when lerobot falls back to the Hub.
            info_path = os.path.join(root, "meta", "info.json")
            if not os.path.isfile(info_path):
                _logger.warning(
                    "NavDP_LerobotV3_Dataset: skipping %s — missing %s",
                    root, info_path,
                )
                skipped_roots.append(root)
                continue
            if not _ensure_tasks_parquet(root):
                _logger.warning(
                    "NavDP_LerobotV3_Dataset: skipping %s — failed to ensure tasks.parquet",
                    root,
                )
                skipped_roots.append(root)
                continue
            if not _ensure_episodes_parquet(root):
                _logger.warning(
                    "NavDP_LerobotV3_Dataset: skipping %s — failed to rebuild episodes parquet "
                    "(missing data/ parquet or zero episodes)",
                    root,
                )
                skipped_roots.append(root)
                continue
            # `revision=""` makes is_valid_version() return False, so even if
            # lerobot ever enters its FileNotFoundError fallback branch it will
            # NOT call get_safe_version() (and hence not contact the Hub).
            try:
                ds = LeRobotDataset(
                    repo_id=f"local/navdp_v3_{ds_idx}",
                    root=root,
                    revision="",
                    download_videos=False,
                    video_backend="pyav",
                )
            except Exception as exc:
                _logger.warning(
                    "NavDP_LerobotV3_Dataset: skipping %s — LeRobotDataset init failed: %s",
                    root, exc,
                )
                skipped_roots.append(root)
                continue
            self._datasets.append(ds)

            # root 是 LeRobotDataset 根目录 (如 robot1/, 包含 meta/, data/, videos/)
            if self.use_pointcloud:
                pcd_path = _resolve_pointcloud_path(Path(root), pointcloud_name)
            else:
                pcd_path = f'scene:{ds_idx}:{root}'
            for ep_idx in range(ds.meta.total_episodes):
                ep = ds.meta.episodes[ep_idx]
                ep_from = int(ep["dataset_from_index"])
                ep_to = int(ep["dataset_to_index"])
                if ep_to - ep_from < 2:
                    # Skip degenerate episodes; NavDP needs >=2 frames.
                    continue
                episode_records.append((ds_idx, ep_idx, ep_from, ep_to, pcd_path))

        if len(episode_records) == 0:
            if skipped_roots:
                raise RuntimeError(
                    f"No usable episodes discovered. {len(skipped_roots)} dataset(s) "
                    f"were skipped due to missing/corrupt metadata:\n"
                    + "\n".join(f"  - {r}" for r in skipped_roots)
                    + f"\nOriginal root_list had {len(root_list)} entries."
                )
            raise RuntimeError(
                f"No usable episodes discovered under: {root_list}"
            )

        # Populate the same containers the parent class expects, but with keys
        # instead of file paths. The parent never inspects their types, it only
        # forwards them to load_image / load_depth / load_pointcloud / read_parquet.
        self.trajectory_data_dir = []
        self.trajectory_rgb_path = []
        self.trajectory_depth_path = []
        self.trajectory_afford_path = []
        for ds_idx, ep_idx, ep_from, ep_to, pcd_path in episode_records:
            ep_len = ep_to - ep_from
            self.trajectory_data_dir.append(("parquet", ds_idx, ep_idx))
            self.trajectory_rgb_path.append(
                [("rgb", ds_idx, ep_idx, t) for t in range(ep_len)]
            )
            self.trajectory_depth_path.append(
                [("depth", ds_idx, ep_idx, t) for t in range(ep_len)]
            )
            self.trajectory_afford_path.append(pcd_path)

        # 使用 dataset_repeat 控制 episode 重复次数
        # 不再强制最小重复次数，数据增强通过其他方式实现
        effective_repeat = self.dataset_repeat
        self.trajectory_data_dir = self.trajectory_data_dir * effective_repeat
        self.trajectory_rgb_path = self.trajectory_rgb_path * effective_repeat
        self.trajectory_depth_path = self.trajectory_depth_path * effective_repeat
        self.trajectory_afford_path = self.trajectory_afford_path * effective_repeat

        # Cache (ds_idx, ep_from) lookup keyed by trajectory index for the
        # parquet / video readers below. Must use the same repeat factor as
        # the trajectory_* lists above.
        self._index_to_ep = [
            (ds_idx, ep_idx, ep_from, ep_to)
            for ds_idx, ep_idx, ep_from, ep_to, _ in episode_records
        ] * effective_repeat

        # Per-episode data cache to avoid repeatedly reading from LeRobotDataset.
        # Key: (ds_idx, ep_idx), Value: (camera_intrinsic, camera_extrinsic, camera_trajectory)
        self._episode_cache: dict[tuple[int, int], tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        # Per-frame decoded video cache. __getitem__ can request the same RGB
        # frame more than once (pixel goal + resized image), and GRPO samples
        # repeatedly from a small scene set per round. Bounded LRU (OrderedDict)
        # evicted by *bytes* (see _FRAME_CACHE_MAX_BYTES) — frames are native
        # resolution float32 (~3.5 MiB each), so a count bound would not protect
        # host RAM.
        self._frame_cache: "OrderedDict[tuple[str, int, int, int], np.ndarray]" = OrderedDict()
        self._frame_cache_bytes = 0
        # Per-scene pointcloud cache. GRPO repeatedly samples episodes from the
        # same selected scenes, so re-reading the PLY for every __getitem__ is
        # pure I/O overhead. Bounded LRU by bytes (see _POINTCLOUD_CACHE_MAX_BYTES)
        # so scene rotation over a long run cannot accumulate every PLY ever read.
        self._pointcloud_cache: "OrderedDict[str, tuple[np.ndarray, np.ndarray]]" = OrderedDict()
        self._pointcloud_cache_bytes = 0

        # ------------------------------------------------------------------
        # Build scene index for GRPO trainer compatibility
        # Note: NavDPTrainer (SFT/IL) does NOT need these; only GRPO requires them.
        # ------------------------------------------------------------------
        self._build_scene_index()

    def _build_scene_index(self):
        """Build scene index: group episodes by scene (afford_path).

        For LeRobotV3 datasets, we use the pointcloud path as the scene identifier.
        Each unique pointcloud path represents a distinct scene.
        """
        num_raw = len(self.trajectory_data_dir) // self.dataset_repeat
        self.trajectory_scene_id = []  # scene_id for each episode (before repeat)
        for idx in range(num_raw):
            # Use pointcloud path as scene identifier
            scene_id = self.trajectory_afford_path[idx]
            self.trajectory_scene_id.append(scene_id)

        # Build episodes_by_scene mapping
        self.episodes_by_scene = {}  # scene_id -> List[episode_index] (raw, no repeat)
        self.unique_scene_ids = []   # deduplicated scene IDs
        for idx in range(num_raw):
            scene_id = self.trajectory_scene_id[idx]
            if scene_id not in self.episodes_by_scene:
                self.episodes_by_scene[scene_id] = []
                self.unique_scene_ids.append(scene_id)
            self.episodes_by_scene[scene_id].append(idx)

    def sample_episodes_from_scenes(self, num_scenes: int, num_episodes: int, rng=None, return_scene_ids=False):
        """Randomly select num_scenes scenes and sample num_episodes from them.

        Args:
            num_scenes: Number of scenes to select
            num_episodes: Total episodes to sample from selected scenes
            rng: numpy random Generator (optional, for reproducibility)
            return_scene_ids: Whether to return the selected scene ID list

        Returns:
            episode_indices: List[int], selected episode indices
            scene_ids (optional): List[str], selected scene IDs (only when return_scene_ids=True)
        """
        if rng is None:
            rng = np.random.default_rng()
        # Randomly select num_scenes scenes
        selected_scenes = rng.choice(
            self.unique_scene_ids,
            size=min(num_scenes, len(self.unique_scene_ids)),
            replace=False
        )
        # Collect all episode indices from these scenes
        candidate_indices = []
        for sid in selected_scenes:
            candidate_indices.extend(self.episodes_by_scene[sid])
        # Randomly sample num_episodes from candidates
        num_episodes = min(num_episodes, len(candidate_indices))
        chosen = rng.choice(candidate_indices, size=num_episodes, replace=False).tolist()

        if return_scene_ids:
            return chosen, selected_scenes.tolist()
        return chosen

    # ------------------------------------------------------------------
    # 特殊轨迹 (掉头 / 大角度拐弯) 索引加载与混合采样
    # ------------------------------------------------------------------

    def load_special_indices(self, index_path: str) -> dict:
        """加载离线扫描生成的特殊轨迹索引 JSON.

        Args:
            index_path: scripts/train/preprocess_special_trajectories.py 输出.

        Returns:
            统计信息 dict.

        Notes:
            * JSON 中的索引是 *raw* (未乘 dataset_repeat) 的 episode idx.
            * 采样返回的 episode_indices 也是 raw idx, 与 collect_episodes_to_buffer
              中的 `self.train_dataset[dataset_idx]` 语义一致 (__getitem__ 直接按
              raw 索引访问 trajectory_data_dir[raw_idx], 而 trajectory_data_dir
              本身已经过 dataset_repeat 扩展, 所以重复 episode 可被正确命中).
            * 若索引文件里的 num_raw_episodes 与当前数据集不一致, 仅打印
              告警, 不强制中断 (允许在子集上调试).
            * schema_version=2 (segment-level): 索引中每项为 {"ep": int, "s": int, "t": int},
              表示 (episode_idx, pixel_start, target) 三元组, 训练时走 __getitem_segment__.
            * schema_version=1 (episode-level, 旧格式): 自动退化, s/t 设为整条 episode
              范围, 但此时 segment 语义与训练采样不一致, 仅作为向后兼容.
        """
        import json as _json

        with open(index_path, "r") as f:
            data = _json.load(f)

        num_raw = len(self.trajectory_data_dir) // self.dataset_repeat
        idx_num_raw = int(data.get("num_raw_episodes", -1))
        if idx_num_raw != num_raw:
            print(
                f"[Special Index][WARN] index 中 num_raw_episodes={idx_num_raw}, "
                f"当前数据集 num_raw={num_raw}; 若不一致建议重新扫描. "
                f"将继续使用 index 文件 (out-of-range 项会被自动丢弃)."
            )

        schema_ver = int(data.get("schema_version", 1))

        # ----------------------------------------------------------------
        # 根据 schema_version 分支加载
        # ----------------------------------------------------------------
        if schema_ver >= 2:
            # ---- v2: segment-level ----
            # 每个 segment = {"ep": int, "s": int, "t": int}
            def _clip_segments(seg_list):
                return [
                    {"ep": int(s["ep"]), "s": int(s["s"]), "t": int(s["t"])}
                    for s in seg_list
                    if 0 <= int(s["ep"]) < num_raw
                ]

            self._uturn_segments: List[Dict] = _clip_segments(data.get("uturn_segments", []))
            self._sharp_turn_segments: List[Dict] = _clip_segments(data.get("sharp_turn_segments", []))
            self._normal_segments: List[Dict] = _clip_segments(data.get("normal_segments", []))

            # 为向后兼容, 仍填充 episode-level 属性 (去重后的 ep idx)
            self.uturn_indices: List[int] = sorted(set(s["ep"] for s in self._uturn_segments))
            self.sharp_turn_indices: List[int] = sorted(set(s["ep"] for s in self._sharp_turn_segments))
            self.normal_indices: List[int] = sorted(set(s["ep"] for s in self._normal_segments))

            # 反向映射: 每个 raw idx -> 类别 (同一 episode 可能同时有 uturn 和 normal
            # segment, 此时优先标记为 special)
            self._episode_label: Dict[int, str] = {}
            for i in self.normal_indices:
                self._episode_label[i] = "normal"
            for i in self.sharp_turn_indices:
                self._episode_label[i] = "sharp_turn"
            for i in self.uturn_indices:
                self._episode_label[i] = "uturn"

            # 按场景分桶 — segment 级
            # scene_id -> {"uturn": [{"ep":..,"s":..,"t":..}, ...], ...}
            self._special_segments_by_scene: Dict[str, Dict[str, List[Dict]]] = {}
            for seg in self._uturn_segments:
                sid = self.trajectory_scene_id[seg["ep"]]
                slot = self._special_segments_by_scene.setdefault(
                    sid, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot["uturn"].append(seg)
            for seg in self._sharp_turn_segments:
                sid = self.trajectory_scene_id[seg["ep"]]
                slot = self._special_segments_by_scene.setdefault(
                    sid, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot["sharp_turn"].append(seg)
            for seg in self._normal_segments:
                sid = self.trajectory_scene_id[seg["ep"]]
                slot = self._special_segments_by_scene.setdefault(
                    sid, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot["normal"].append(seg)

            # 旧接口兼容: episode 级分桶 (trainer 中可能直接引用)
            self._special_by_scene: Dict[str, Dict[str, List[int]]] = {}
            for raw_idx, label in self._episode_label.items():
                sid = self.trajectory_scene_id[raw_idx]
                slot = self._special_by_scene.setdefault(
                    sid, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot[label].append(raw_idx)

            stats = {
                "schema_version": 2,
                "num_uturn_segments": len(self._uturn_segments),
                "num_sharp_turn_segments": len(self._sharp_turn_segments),
                "num_normal_segments": len(self._normal_segments),
                "num_uturn_episodes": len(self.uturn_indices),
                "num_sharp_turn_episodes": len(self.sharp_turn_indices),
                "num_normal_episodes": len(self.normal_indices),
                "num_scenes_with_special": sum(
                    1 for s in self._special_segments_by_scene.values()
                    if (len(s["uturn"]) + len(s["sharp_turn"])) > 0
                ),
            }
            print(
                f"[Special Index] Loaded v2 (segment-level) {index_path}: "
                f"uturn_seg={stats['num_uturn_segments']} sharp_turn_seg={stats['num_sharp_turn_segments']} "
                f"normal_seg={stats['num_normal_segments']} "
                f"(episodes: uturn={stats['num_uturn_episodes']} sharp_turn={stats['num_sharp_turn_episodes']} "
                f"normal={stats['num_normal_episodes']} "
                f"scenes_with_special={stats['num_scenes_with_special']})"
            )
        else:
            # ---- v1: episode-level (旧格式) ----
            def _clip(idx_list):
                return [int(i) for i in idx_list if 0 <= int(i) < num_raw]

            self.uturn_indices: List[int] = _clip(data.get("uturn_indices", []))
            self.sharp_turn_indices: List[int] = _clip(data.get("sharp_turn_indices", []))
            self.normal_indices: List[int] = _clip(data.get("normal_indices", []))

            self._episode_label: Dict[int, str] = {}
            for i in self.normal_indices:
                self._episode_label[i] = "normal"
            for i in self.sharp_turn_indices:
                self._episode_label[i] = "sharp_turn"
            for i in self.uturn_indices:
                self._episode_label[i] = "uturn"

            # episode 级分桶
            self._special_by_scene: Dict[str, Dict[str, List[int]]] = {}
            for raw_idx, label in self._episode_label.items():
                scene_id = self.trajectory_scene_id[raw_idx]
                slot = self._special_by_scene.setdefault(
                    scene_id, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot[label].append(raw_idx)

            # v1 没有真正的 segment, 构造退化的 segment 列表 (整条 episode 范围)
            # 这使得 sample_mixed_episodes_from_scenes 在 v1 下仍能工作,
            # 但 segment 的 s/t 不可用, 采样时会退回到 __getitem__ (不指定 segment).
            self._uturn_segments: List[Dict] = [
                {"ep": ep, "s": -1, "t": -1} for ep in self.uturn_indices
            ]
            self._sharp_turn_segments: List[Dict] = [
                {"ep": ep, "s": -1, "t": -1} for ep in self.sharp_turn_indices
            ]
            self._normal_segments: List[Dict] = [
                {"ep": ep, "s": -1, "t": -1} for ep in self.normal_indices
            ]
            self._special_segments_by_scene: Dict[str, Dict[str, List[Dict]]] = {}
            for seg in self._uturn_segments:
                sid = self.trajectory_scene_id[seg["ep"]]
                slot = self._special_segments_by_scene.setdefault(
                    sid, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot["uturn"].append(seg)
            for seg in self._sharp_turn_segments:
                sid = self.trajectory_scene_id[seg["ep"]]
                slot = self._special_segments_by_scene.setdefault(
                    sid, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot["sharp_turn"].append(seg)
            for seg in self._normal_segments:
                sid = self.trajectory_scene_id[seg["ep"]]
                slot = self._special_segments_by_scene.setdefault(
                    sid, {"uturn": [], "sharp_turn": [], "normal": []}
                )
                slot["normal"].append(seg)

            stats = {
                "schema_version": 1,
                "num_uturn": len(self.uturn_indices),
                "num_sharp_turn": len(self.sharp_turn_indices),
                "num_normal": len(self.normal_indices),
                "num_scenes_with_special": sum(
                    1 for s in self._special_by_scene.values()
                    if (len(s["uturn"]) + len(s["sharp_turn"])) > 0
                ),
            }
            print(
                f"[Special Index] Loaded v1 (episode-level, legacy) {index_path}: "
                f"uturn={stats['num_uturn']} sharp_turn={stats['num_sharp_turn']} "
                f"normal={stats['num_normal']} "
                f"(scenes_with_special={stats['num_scenes_with_special']})"
            )

        return stats

    @property
    def has_special_indices(self) -> bool:
        """是否已加载特殊轨迹索引."""
        return hasattr(self, "_special_by_scene")

    def get_episode_label(self, raw_idx: int) -> str:
        """返回 raw_idx 对应的类别. 未加载索引或未登记时返回 "normal"."""
        if not self.has_special_indices:
            return "normal"
        return self._episode_label.get(int(raw_idx), "normal")

    def sample_mixed_episodes_from_scenes(
        self,
        num_scenes: int,
        num_episodes: int,
        uturn_ratio: float = 0.0,
        sharp_turn_ratio: float = 0.0,
        rng=None,
        allow_cross_scene_fallback: bool = True,
        return_scene_ids: bool = False,
    ):
        """按比例混入 uturn / sharp_turn segment, 其余为 normal 随机采样.

        关键设计:
            * 仍保留 "num_scenes_per_buffer 个场景" 的约束: normal 部分
              完全来自这些场景, pointcloud 缓存命中率不下降.
            * uturn / sharp_turn 优先在已选场景内取; 若不足, 且
              allow_cross_scene_fallback=True, 才跨场景补齐 (此时该轮
              新增的场景的 pointcloud 会被加载, 但只影响特殊样本部分).
            * 比例之和 > 1 时会被截断: 实际 normal 数 = max(0, total - special).
            * v2 (segment-level): 采样粒度为 segment {"ep","s","t"},
              训练循环用 __getitem_segment__(ep, s, t) 加载.
            * v1 (episode-level, 旧格式): s=-1, t=-1, 训练循环回退到
              __getitem__(ep).

        Returns:
            sampled (list[tuple]):
                每项 = (ep_idx, segment_or_None)
                - special: (ep_idx, {"ep":.., "s":.., "t":..})
                - normal:  (ep_idx, None)
            scene_ids (list[str]) - 仅当 return_scene_ids=True
            counts (dict) - {"normal": n, "uturn": u, "sharp_turn": s, "fallback": f,
                             "total": t}
        """
        if not self.has_special_indices:
            raise RuntimeError(
                "尚未调用 load_special_indices(), 不能使用 sample_mixed_episodes_from_scenes."
            )
        if rng is None:
            rng = np.random.default_rng()

        # 1. 决定各类目标数量
        uturn_ratio = max(0.0, float(uturn_ratio))
        sharp_turn_ratio = max(0.0, float(sharp_turn_ratio))
        if uturn_ratio + sharp_turn_ratio > 1.0:
            scale = 1.0 / (uturn_ratio + sharp_turn_ratio)
            uturn_ratio *= scale
            sharp_turn_ratio *= scale
        target_uturn = int(round(num_episodes * uturn_ratio))
        target_sharp = int(round(num_episodes * sharp_turn_ratio))
        target_special_total = target_uturn + target_sharp
        target_normal = max(0, num_episodes - target_special_total)

        # 2. 选 num_scenes 个场景 (优先选包含特殊 segment 的场景)
        scenes_with_special = [
            sid for sid, s in self._special_segments_by_scene.items()
            if (target_uturn > 0 and len(s["uturn"]) > 0)
            or (target_sharp > 0 and len(s["sharp_turn"]) > 0)
        ]
        other_scenes = [
            sid for sid in self.unique_scene_ids if sid not in set(scenes_with_special)
        ]

        n_total = min(num_scenes, len(self.unique_scene_ids))
        n_priority = min(len(scenes_with_special), n_total)
        # 让一部分名额给 normal-only 场景, 避免特殊场景过度集中
        n_priority = min(n_priority, max(1, n_total // 2)) if scenes_with_special and other_scenes else n_priority
        selected_priority = (
            rng.choice(scenes_with_special, size=n_priority, replace=False).tolist()
            if n_priority > 0 else []
        )
        n_other = n_total - len(selected_priority)
        if n_other > 0 and len(other_scenes) > 0:
            n_other = min(n_other, len(other_scenes))
            selected_other = rng.choice(other_scenes, size=n_other, replace=False).tolist()
        else:
            selected_other = []
        selected_scenes: List[str] = list(selected_priority) + list(selected_other)

        # 3. 在已选场景内收集 segment 池 (special) 和 episode 池 (normal)
        def _draw_segs(pool: List[Dict], k: int):
            """从 segment 池中不重复抽取 k 个."""
            if k <= 0 or len(pool) == 0:
                return []
            k = min(k, len(pool))
            return rng.choice(pool, size=k, replace=False).tolist()

        def _draw_eps(pool: List[int], k: int):
            """从 episode idx 池中不重复抽取 k 个."""
            if k <= 0 or len(pool) == 0:
                return []
            k = min(k, len(pool))
            return rng.choice(pool, size=k, replace=False).tolist()

        in_scene_uturn_segs: List[Dict] = []
        in_scene_sharp_segs: List[Dict] = []
        in_scene_normal_eps: List[int] = []
        for sid in selected_scenes:
            seg_slot = self._special_segments_by_scene.get(sid)
            if seg_slot is None:
                # 此场景在索引中没有登记, 全部视为 normal
                in_scene_normal_eps.extend(self.episodes_by_scene.get(sid, []))
                continue
            in_scene_uturn_segs.extend(seg_slot["uturn"])
            in_scene_sharp_segs.extend(seg_slot["sharp_turn"])
            # normal 用 episodes_by_scene (raw idx 列表), 不走 segment 池
            in_scene_normal_eps.extend(self.episodes_by_scene.get(sid, []))

        chosen_uturn_segs: List[Dict] = _draw_segs(in_scene_uturn_segs, target_uturn)
        chosen_sharp_segs: List[Dict] = _draw_segs(in_scene_sharp_segs, target_sharp)

        # 4. 不足时跨场景补齐 (从全局 segment 池中补充)
        fallback_count = 0
        if allow_cross_scene_fallback:
            # 收集已选 segment 的 ep+start 唯一标识, 避免重复
            chosen_seg_ids = set()
            for seg in chosen_uturn_segs + chosen_sharp_segs:
                chosen_seg_ids.add((seg["ep"], seg["s"], seg["t"]))

            if len(chosen_uturn_segs) < target_uturn:
                deficit = target_uturn - len(chosen_uturn_segs)
                extra_pool = [s for s in self._uturn_segments
                              if (s["ep"], s["s"], s["t"]) not in chosen_seg_ids]
                extra = _draw_segs(extra_pool, deficit)
                chosen_uturn_segs.extend(extra)
                fallback_count += len(extra)
                chosen_seg_ids.update((s["ep"], s["s"], s["t"]) for s in extra)
                for s in extra:
                    sid = self.trajectory_scene_id[s["ep"]]
                    if sid not in selected_scenes:
                        selected_scenes.append(sid)

            if len(chosen_sharp_segs) < target_sharp:
                deficit = target_sharp - len(chosen_sharp_segs)
                extra_pool = [s for s in self._sharp_turn_segments
                              if (s["ep"], s["s"], s["t"]) not in chosen_seg_ids]
                extra = _draw_segs(extra_pool, deficit)
                chosen_sharp_segs.extend(extra)
                fallback_count += len(extra)
                chosen_seg_ids.update((s["ep"], s["s"], s["t"]) for s in extra)
                for s in extra:
                    sid = self.trajectory_scene_id[s["ep"]]
                    if sid not in selected_scenes:
                        selected_scenes.append(sid)

        # 5. 用 normal episode 补齐
        # normal 不需要指定 segment, 从 episodes_by_scene 中随机抽
        # 注意: 排除 chosen_special_eps 是因为同一 episode 不应同时以 segment 和
        # random 两种方式出现 (否则 buffer 中同一 episode 的数据重复, 导致
        # GRPO 优势计算偏向该 episode).
        chosen_special_eps = set(s["ep"] for s in chosen_uturn_segs + chosen_sharp_segs)
        normal_pool = [i for i in in_scene_normal_eps if i not in chosen_special_eps]
        chosen_normal_eps: List[int] = _draw_eps(normal_pool, target_normal)
        # 若 normal 不够 (场景太小), 在已选场景的任意 episode 中补
        # 此处仅排除已选 normal 和 special, 允许同一 episode 被选多次
        if len(chosen_normal_eps) < target_normal:
            deficit = target_normal - len(chosen_normal_eps)
            remaining = [
                i for i in (in_scene_normal_eps)
                if i not in chosen_special_eps and i not in set(chosen_normal_eps)
            ]
            chosen_normal_eps.extend(_draw_eps(remaining, deficit))

        # 6. 组装返回列表: (ep_idx, segment_or_None)
        #    - special: (ep, {"ep":.., "s":.., "t":..}) — 保留完整 segment 信息
        #    - normal:  (ep, None) — 训练时走 __getitem__(ep)
        result: List[Tuple[int, Optional[Dict]]] = []
        for seg in chosen_uturn_segs:
            result.append((int(seg["ep"]), seg))
        for seg in chosen_sharp_segs:
            result.append((int(seg["ep"]), seg))
        for ep in chosen_normal_eps:
            result.append((int(ep), None))
        rng.shuffle(result)

        counts = {
            "normal": len(chosen_normal_eps),
            "uturn": len(chosen_uturn_segs),
            "sharp_turn": len(chosen_sharp_segs),
            "fallback": int(fallback_count),
            "total": len(result),
        }
        if return_scene_ids:
            return result, selected_scenes, counts
        return result, counts

    # ------------------------------------------------------------------
    # I/O hooks overriding NavDP_Base_Datset
    # ------------------------------------------------------------------

    def _resolve_key(self, key):
        """Map a (kind, ds_idx, ep_idx, t) key to the underlying LeRobotDataset
        and global frame index. Returns (lerobot_dataset, global_frame_idx)."""
        if not (isinstance(key, tuple) and len(key) == 4):
            raise TypeError(
                f"NavDP_LerobotV3_Dataset expected a 4-tuple key, got: {key!r}"
            )
        _kind, ds_idx, ep_idx, t = key
        ds = self._datasets[ds_idx]
        ep_from = int(ds.meta.episodes[ep_idx]["dataset_from_index"])
        return ds, ep_from + int(t)

    @staticmethod
    def _ensure_lerobot_loaded(ds):
        """Load the underlying HF dataset across LeRobot API versions."""
        if hasattr(ds, "_ensure_hf_dataset_loaded"):
            ds._ensure_hf_dataset_loaded()
            return getattr(ds, "hf_dataset", None)

        # LeRobot v3 exposes `hf_dataset` as a lazy-loading property. Accessing
        # it creates/activates the reader when needed.
        return ds.hf_dataset

    @staticmethod
    def _query_lerobot_videos(ds, query_timestamps, ep_idx):
        """Decode video frames with PyAV, bypassing lerobot's video backend
        (which requires torchcodec or torchvision.io.VideoReader)."""
        import av
        ep = ds.meta.episodes[ep_idx]
        result = {}
        for vid_key, query_ts in query_timestamps.items():
            from_ts = ep[f"videos/{vid_key}/from_timestamp"]
            shifted_ts = [from_ts + ts for ts in query_ts]
            video_path = str(ds.root / ds.meta.get_video_file_path(ep_idx, vid_key))
            result[vid_key] = _decode_frames_pyav(video_path, shifted_ts, ds.tolerance_s)
        return result

    def _cache_get(self, cache_key):
        """LRU read: return cached frame and mark it most-recently-used."""
        cached = self._frame_cache.get(cache_key)
        if cached is not None:
            self._frame_cache.move_to_end(cache_key)
        return cached

    def _cache_put(self, cache_key, frame):
        """LRU write bounded by total bytes: evict least-recently-used until the
        cache fits within _FRAME_CACHE_MAX_BYTES (frames are large, so we budget
        memory rather than entry count)."""
        existing = self._frame_cache.pop(cache_key, None)
        if existing is not None:
            self._frame_cache_bytes -= getattr(existing, "nbytes", 0)
        self._frame_cache[cache_key] = frame
        self._frame_cache_bytes += getattr(frame, "nbytes", 0)
        # Evict oldest until within budget, but always keep the just-added entry.
        while self._frame_cache_bytes > _FRAME_CACHE_MAX_BYTES and len(self._frame_cache) > 1:
            _, evicted = self._frame_cache.popitem(last=False)
            self._frame_cache_bytes -= getattr(evicted, "nbytes", 0)

    def _safe_ep_from(self, ds_idx: int, ep_idx: int):
        """Look up dataset_from_index with bounds checking.

        Returns (ep_from, ep_to) or raises IndexError if ep_idx is out of
        bounds for the current episodes list.
        """
        ds = self._datasets[ds_idx]
        if ep_idx >= len(ds.meta.episodes):
            raise IndexError(
                f"ep_idx {ep_idx} out of bounds for ds {ds_idx} "
                f"(episodes has {len(ds.meta.episodes)} rows)"
            )
        ep = ds.meta.episodes[ep_idx]
        return int(ep["dataset_from_index"]), int(ep["dataset_to_index"])

    def _prefetch_video_frames(self, kind: str, ds_idx: int, ep_idx: int, ts_list):
        """Decode several frames of one (kind, ds_idx, ep_idx) in a single pass.

        lerobot's `decode_video_frames` selects the closest frame to each query
        timestamp independently (argmin) and stacks them in query order, so a
        batched decode yields byte-identical frames to decoding each timestamp on
        its own — only the seek/scan is amortised. Frames are written straight
        into `_frame_cache`, so the inherited per-frame `process_image` calls that
        follow all hit the cache.
        """
        ds = self._datasets[ds_idx]
        hf_dataset = self._ensure_lerobot_loaded(ds)
        ep_from, ep_to = self._safe_ep_from(ds_idx, ep_idx)
        feature_key = self.rgb_key if kind == "rgb" else self.depth_key

        # Only decode frames not already cached; keep request order stable.
        missing_t = []
        for t in ts_list:
            t = int(t)
            if (kind, int(ds_idx), int(ep_idx), t) not in self._frame_cache:
                missing_t.append(t)
        if not missing_t:
            return
        # De-duplicate while preserving order.
        missing_t = list(dict.fromkeys(missing_t))

        timestamps = []
        for t in missing_t:
            global_idx = ep_from + t
            if global_idx < 0 or global_idx >= len(hf_dataset):
                raise IndexError(
                    f"global_idx {global_idx} out of bounds for ds {ds_idx} "
                    f"(hf_dataset has {len(hf_dataset)} rows, ep_from={ep_from}, t={t})"
                )
            ts_raw = hf_dataset[global_idx]["timestamp"]
            timestamps.append(float(ts_raw.item() if isinstance(ts_raw, torch.Tensor) else ts_raw))

        frames = self._query_lerobot_videos(ds, {feature_key: timestamps}, int(ep_idx))[feature_key]
        if isinstance(frames, torch.Tensor):
            frames = frames.detach().cpu().numpy()
        # A single-timestamp query is squeezed to (C, H, W) by lerobot; a
        # multi-timestamp query stays (N, C, H, W). Normalise to per-frame writes.
        if len(missing_t) == 1:
            self._cache_put((kind, int(ds_idx), int(ep_idx), missing_t[0]), frames)
        else:
            for t, frame in zip(missing_t, frames):
                self._cache_put((kind, int(ds_idx), int(ep_idx), int(t)), frame)

    def _get_video_frame(self, kind: str, ds_idx: int, ep_idx: int, t: int):
        """Decode one video frame with a bounded LRU cache, bypassing numeric columns."""
        cache_key = (kind, int(ds_idx), int(ep_idx), int(t))
        cached = self._cache_get(cache_key)
        if cached is not None:
            return cached

        ds = self._datasets[ds_idx]
        hf_dataset = self._ensure_lerobot_loaded(ds)
        ep_from, ep_to = self._safe_ep_from(ds_idx, ep_idx)
        global_idx = ep_from + int(t)
        if global_idx < 0 or global_idx >= len(hf_dataset):
            raise IndexError(
                f"global_idx {global_idx} out of bounds for ds {ds_idx} "
                f"(hf_dataset has {len(hf_dataset)} rows, ep_from={ep_from}, t={t})"
            )
        timestamp_raw = hf_dataset[global_idx]["timestamp"]
        timestamp = float(timestamp_raw.item() if isinstance(timestamp_raw, torch.Tensor) else timestamp_raw)
        feature_key = self.rgb_key if kind == "rgb" else self.depth_key
        frame = self._query_lerobot_videos(ds, {feature_key: [timestamp]}, int(ep_idx))[feature_key]
        if isinstance(frame, torch.Tensor):
            frame = frame.detach().cpu().numpy()
        self._cache_put(cache_key, frame)
        return frame

    def load_image(self, image_url):
        """Return an HxWx3 uint8 ndarray. Accepts our tuple key only."""
        _kind, ds_idx, ep_idx, t = image_url
        frame = self._get_video_frame("rgb", ds_idx, ep_idx, t)  # (C, H, W) float32 in [0, 1]
        # CHW float [0,1] -> HWC uint8 [0,255]
        frame = np.transpose(frame, (1, 2, 0))
        frame = np.clip(frame * 255.0, 0.0, 255.0).astype(np.uint8)
        return frame

    def load_depth(self, depth_url):
        """Return an HxW uint16 ndarray whose unit matches `load_depth/10000=m`,
        so the inherited process_depth keeps working unchanged."""
        _kind, ds_idx, ep_idx, t = depth_url
        frame = self._get_video_frame("depth", ds_idx, ep_idx, t)  # (3, H, W) float32 [0,1]
        # Convert (C, H, W) normalised depth back to uint16 millimetres-x10
        # (i.e. depth_m * 10000) so that parent's `/ 10000.0` recovers metres.
        depth_norm = frame[0]  # all three channels are identical
        depth_units = np.clip(
            depth_norm * MAX_DEPTH_M * 10000.0, 0.0, 65535.0
        ).astype(np.uint16)
        return depth_units

    def process_memory(self, rgb_paths, depth_paths, start_step, memory_digit=1):
        """Batch-decode the memory-window RGB frames, then defer to the parent.

        The inherited `process_memory` loads each memory frame with a separate
        `process_image -> _get_video_frame -> _query_videos` call, i.e. one video
        seek per frame. Here we pre-compute the same `memory_index` the parent
        uses and decode all those RGB frames (plus the depth frame) in one
        `_query_videos` pass each. The parent then runs unchanged and every
        per-frame read is a cache hit. Frame selection is byte-identical to the
        per-frame path, so the returned tensors are unchanged.
        """
        # Mirror the parent's memory_index computation exactly: the arange is
        # monotonically increasing, so the negative entries are exactly the
        # leading ones the parent drops with `[outrange_sum:]`.
        memory_index = np.arange(
            start_step - (self.memory_size - 1) * memory_digit,
            start_step + 1,
            memory_digit,
        )
        outrange_sum = int((memory_index < 0).sum())
        memory_index = memory_index[outrange_sum:]

        # Clip memory_index and start_step to the actual path length.
        # Episodes metadata can report more frames than rgb_paths has
        # tuple keys (or fewer after a _getitem_impl resync), so we
        # guard against "list index out of range" here.
        path_len = len(rgb_paths)
        if path_len > 0:
            memory_index = memory_index[memory_index < path_len]
            start_step = min(int(start_step), path_len - 1)
        else:
            raise IndexError("rgb_paths is empty — degenerate episode")

        # Prefetch RGB frames for the in-range memory indices in a single decode.
        try:
            keys = [rgb_paths[int(i)] for i in memory_index]
            if keys and isinstance(keys[0], tuple) and len(keys[0]) == 4:
                kind, ds_idx, ep_idx, _ = keys[0]
                self._prefetch_video_frames(
                    kind, ds_idx, ep_idx, [k[3] for k in keys]
                )
            # The depth frame the parent reads is depth_paths[start_step].
            dkey = depth_paths[int(start_step)]
            if isinstance(dkey, tuple) and len(dkey) == 4:
                self._prefetch_video_frames(dkey[0], dkey[1], dkey[2], [dkey[3]])
        except Exception as e:  # never let a prefetch failure break loading
            print(f"process_memory prefetch skipped: {e}")

        return super().process_memory(rgb_paths, depth_paths, start_step, memory_digit=memory_digit)

    def _skip_corrupt_episode(self, index, reason):
        """Log corrupt episode and resample process_data_parquet only."""
        import logging
        _logger = logging.getLogger(__name__)
        ds_idx, ep_idx, ep_from, ep_to = self._index_to_ep[index]
        _logger.warning(
            "NavDP_LerobotV3_Dataset: %s for ds=%d ep=%d [%d:%d] — resampling parquet",
            reason, ds_idx, ep_idx, ep_from, ep_to,
        )
        alt = np.random.randint(len(self._index_to_ep))
        return self.process_data_parquet(alt)

    def _skip_corrupt_sample(self, index, context):
        """Log corrupt sample and resample the entire __getitem__.

        Corrupt episodes can fail at any depth (parquet read, video
        decode, memory load, etc.).  Resampling the full sample ensures
        all per-episode state (paths, indices, pointclouds) is consistent.
        """
        import logging
        _logger = logging.getLogger(__name__)
        ds_idx, ep_idx, ep_from, ep_to = self._index_to_ep[index]
        _logger.warning(
            "NavDP_LerobotV3_Dataset: %s IndexError for ds=%d ep=%d [%d:%d] — full resample",
            context, ds_idx, ep_idx, ep_from, ep_to,
        )
        alt = np.random.randint(len(self._index_to_ep))
        return self.__getitem__(alt)

    def process_data_parquet(self, index):
        """Read episode metadata from the underlying LeRobotDataset with caching.

        This method avoids direct access to `hf_dataset` and instead uses the
        public `LeRobotDataset.__getitem__` API to fetch rows one by one. If
        any local read fails, the error propagates immediately without falling
        back to remote Hub calls. Results are cached per-episode to avoid
        repeated slow decoding.
        """
        ds_idx, ep_idx, ep_from, ep_to = self._index_to_ep[index]
        cache_key = (ds_idx, ep_idx)
        if cache_key in self._episode_cache:
            camera_intrinsic, camera_extrinsic, camera_trajectory = self._episode_cache[cache_key]
            trajectory_length = camera_trajectory.shape[0]
            return camera_intrinsic, camera_extrinsic, camera_trajectory, trajectory_length

        ds = self._datasets[ds_idx]

        # Read numeric columns directly from the HF dataset. Calling
        # LeRobotDataset.__getitem__ here would also decode video frames for
        # every row in the episode, which dominates GRPO collection time.
        hf_dataset = self._ensure_lerobot_loaded(ds)

        # Guard against empty or degenerate slices that would crash inside
        # lerobot's hf_transform_to_torch (it does items_dict[key][0] which
        # raises IndexError on an empty list).
        if ep_from >= ep_to:
            return self._skip_corrupt_episode(index, "empty slice")

        # hf_dataset[slice] triggers lerobot's hf_transform_to_torch
        # internally, which crashes with IndexError if any column has
        # zero rows.  Catch the exception here and resample.
        try:
            rows = hf_dataset[ep_from:ep_to]
        except IndexError:
            return self._skip_corrupt_episode(index, "lerobot transform IndexError")

        # Use the first frame's intrinsic/extrinsic (they are constant per episode).
        intrinsic_raw = rows["observation.camera_intrinsic"][0]
        extrinsic_raw = rows["observation.camera_extrinsic"][0]
        if isinstance(intrinsic_raw, torch.Tensor):
            intrinsic_raw = intrinsic_raw.detach().cpu().numpy()
        if isinstance(extrinsic_raw, torch.Tensor):
            extrinsic_raw = extrinsic_raw.detach().cpu().numpy()

        # Stack actions across frames.
        action_raw = np.stack([
            a.detach().cpu().numpy() if isinstance(a, torch.Tensor) else np.asarray(a)
            for a in rows["action"]
        ], axis=0)

        camera_intrinsic = intrinsic_raw.reshape(3, 3).astype(np.float64)
        camera_extrinsic = extrinsic_raw.reshape(4, 4).astype(np.float64)
        camera_trajectory = action_raw.reshape(-1, 4, 4).astype(np.float64)

        # Cache for future access.
        self._episode_cache[cache_key] = (camera_intrinsic, camera_extrinsic, camera_trajectory)

        trajectory_length = int(camera_trajectory.shape[0])
        return camera_intrinsic, camera_extrinsic, camera_trajectory, trajectory_length

    def process_obstacle_points(self, index):
        """Load raw pointcloud obstacles.

        Returns:
            scene_obstacle_points: [N, 3] 世界坐标系下的真实障碍物点
            scene_inflation_points: [M, 3] 世界坐标系下的膨胀障碍物点
        
        Raises:
            FileNotFoundError: 如果点云文件不存在
        """
        if not self.use_pointcloud:
            empty = np.zeros((0, 3), dtype=np.float32)
            return empty, empty
        path = self.trajectory_afford_path[index]
        if not (isinstance(path, str) and os.path.isfile(path)):
            if not getattr(self, '_warned_missing_pcd', False):
                print(
                    f'NavDP_LerobotV3_Dataset: 点云文件不存在 ({path}); '
                    'SFT 将使用空障碍物点云继续训练。'
                )
                self._warned_missing_pcd = True
            empty = np.zeros((0, 3), dtype=np.float32)
            return empty, empty
        cached = self._pointcloud_cache.get(path)
        if cached is not None:
            self._pointcloud_cache.move_to_end(path)
            return cached

        obstacles, inflation = super().process_obstacle_points(index)
        entry_bytes = getattr(obstacles, "nbytes", 0) + getattr(inflation, "nbytes", 0)
        self._pointcloud_cache[path] = (obstacles, inflation)
        self._pointcloud_cache_bytes += entry_bytes
        # Evict least-recently-used scenes until within the byte budget, but keep
        # the entry we just inserted.
        while self._pointcloud_cache_bytes > _POINTCLOUD_CACHE_MAX_BYTES and len(self._pointcloud_cache) > 1:
            _, (ev_obs, ev_inf) = self._pointcloud_cache.popitem(last=False)
            self._pointcloud_cache_bytes -= getattr(ev_obs, "nbytes", 0) + getattr(ev_inf, "nbytes", 0)
        return self._pointcloud_cache[path]

    # ------------------------------------------------------------------
    # process_pixel_goal: parent's body verbatim, only the raw image read
    # is rerouted through `self.load_image` so it accepts our tuple keys.
    # All projection / mask / pad / resize math is preserved unchanged.
    # ------------------------------------------------------------------

    def process_pixel_goal(self, image_url, target_point, camera_intrinsic, camera_extrinsic):
        # I/O adapter: route raw RGB read through load_image (handles tuple keys).
        image = self.load_image(image_url)
        resize_image = self.process_image(image_url)

        coordinate = np.array([-target_point[1], target_point[0], camera_extrinsic[2, 3] * 0.8])
        camera_coordinate = np.matmul(camera_extrinsic[0:3, 0:3], coordinate[:, None])

        # Avoid division by zero when camera_coordinate[2] is zero (target lies on the camera plane).
        eps = 1e-6
        z_val = camera_coordinate[2, 0] if camera_coordinate[2, 0] != 0 else eps
        pixel_coord_x = float(camera_intrinsic[0, 2] + (camera_coordinate[0, 0] / z_val) * camera_intrinsic[0, 0])
        pixel_coord_y = float(camera_intrinsic[1, 2] + (-camera_coordinate[1, 0] / z_val) * camera_intrinsic[1, 1])

        pixel_mask = np.zeros_like(image)
        visible_flag = False

        # Only render the pixel marker when the projection is valid (z > 0 means in front of camera).
        if (
            camera_coordinate[2, 0] > 0
            and pixel_coord_x > 0
            and pixel_coord_x < image.shape[1]
            and pixel_coord_y > 0
            and pixel_coord_y < image.shape[0]
        ):
            pixel_mask = cv2.rectangle(
                pixel_mask,
                (int(pixel_coord_x - np.random.randint(6, 12)), int(pixel_coord_y - np.random.randint(6, 12))),
                (int(pixel_coord_x + np.random.randint(6, 12)), int(pixel_coord_y + np.random.randint(6, 12))),
                (255, 255, 255),
                -1,
            )
            visible_flag = True

        H, W, C = pixel_mask.shape
        prop = self.image_size / max(H, W)
        pixel_mask = cv2.resize(pixel_mask, (-1, -1), fx=prop, fy=prop)
        pad_width = max((self.image_size - pixel_mask.shape[1]) // 2, 0)
        pad_height = max((self.image_size - pixel_mask.shape[0]) // 2, 0)
        pad_mask = np.pad(
            pixel_mask, ((pad_height, pad_height), (pad_width, pad_width), (0, 0)), mode='constant', constant_values=0
        )
        mask = cv2.resize(pad_mask, (self.image_size, self.image_size), interpolation=cv2.INTER_NEAREST)
        mask = np.array(mask, np.float32) / 255.0
        mask = mask.mean(axis=-1)[:, :, None]
        return np.concatenate((resize_image, mask), axis=-1), visible_flag

    # ------------------------------------------------------------------
    # Coordinate frame adapter: v3 y-forward -> NavDP x-forward
    # ------------------------------------------------------------------

    def relative_pose(self, R_base, T_base, R_world, T_world, base_extrinsic):
        """Convert v3 y-forward local XY labels into NavDP's x-forward frame.

        The legacy NavDP source returns local forward motion on +X after the
        parent `relative_pose` conversion. AgentWorld/LeRobot v3 trajectories
        return the same forward motion on -Y. Mixed training then sees two
        incompatible label frames and the diffusion target becomes multi-modal.
        Rotate the local translation by +90 degrees so v3 samples share the
        legacy NavDP convention.
        """
        R_frame, T_frame = super().relative_pose(R_base, T_base, R_world, T_world, base_extrinsic)
        T_frame = np.array(T_frame, copy=True)
        if T_frame.ndim == 1:
            # 单点: T_frame[0], T_frame[1] = -T_frame[1], T_frame[0]
            # 旋转+90度: x' = -y, y' = x
            x_old = T_frame[0]
            y_old = T_frame[1]
            T_frame[0] = -y_old
            T_frame[1] = x_old
        else:
            # 多点: 每行进行相同的旋转
            x_old = T_frame[:, 0].copy()
            y_old = T_frame[:, 1].copy()
            T_frame[:, 0] = -y_old
            T_frame[:, 1] = x_old
        return R_frame, T_frame

    def absolute_pose(self, R_base, T_base, R_frame, T_frame, base_extrinsic):
        """Inverse of the v3 local-frame adapter in `relative_pose`.

        将 NavDP x-forward 坐标系下的局部坐标转换回 v3 的 y-forward 坐标系，
        然后再调用父类的 absolute_pose 转换到世界坐标系。

        逆旋转-90度: x' = y, y' = -x
        """
        T_frame = np.array(T_frame, copy=True)
        if T_frame.ndim == 1:
            x_old = T_frame[0]
            y_old = T_frame[1]
            T_frame[0] = y_old
            T_frame[1] = -x_old
        else:
            x_old = T_frame[:, 0].copy()
            y_old = T_frame[:, 1].copy()
            T_frame[:, 0] = y_old
            T_frame[:, 1] = -x_old
        return super().absolute_pose(R_base, T_base, R_frame, T_frame, base_extrinsic)

    @staticmethod
    def _window_continuity_mask(bad_transition: np.ndarray, start_idx: np.ndarray, end_idx: np.ndarray) -> np.ndarray:
        """True 表示窗口 [start, end] 内不存在坏过渡。"""
        bad = bad_transition.astype(np.int64)
        csum = np.r_[0, np.cumsum(bad)]
        start = np.asarray(start_idx, dtype=np.int64)
        end = np.asarray(end_idx, dtype=np.int64)
        start_transition = np.minimum(start + 1, bad.shape[0])
        end_transition = np.minimum(end + 1, bad.shape[0])
        return (csum[end_transition] - csum[start_transition]) == 0

    @staticmethod
    def _frame_continuity_mask(bad_transition: np.ndarray) -> np.ndarray:
        """过滤掉坏过渡相邻帧。"""
        n = bad_transition.shape[0]
        clean = ~bad_transition.astype(bool)
        if n > 1:
            clean[:-1] &= ~bad_transition[1:]
        return clean

    def _transition_quality_masks(self, trajectory_extrinsics: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """基于位姿连续性构建坏过渡与有效帧掩码。"""
        xy = trajectory_extrinsics[:, :2, 3].astype(np.float32)
        heading = trajectory_extrinsics[:, :2, 0].astype(np.float32)
        yaw = np.unwrap(np.arctan2(heading[:, 1], heading[:, 0]))
        step = np.linalg.norm(np.diff(xy, axis=0, prepend=xy[:1]), axis=1)
        yaw_delta = np.abs(np.arctan2(np.sin(np.diff(yaw, prepend=yaw[:1])), np.cos(np.diff(yaw, prepend=yaw[:1])))).astype(
            np.float32
        )

        intrinsic_bad_transition = (
            (step > self.max_continuous_step_m)
            | (yaw_delta > self.max_continuous_yaw_rad)
            | ~np.isfinite(step)
            | ~np.isfinite(yaw_delta)
        )
        intrinsic_bad_transition[0] = False
        current_valid = self._frame_continuity_mask(intrinsic_bad_transition)
        return intrinsic_bad_transition, current_valid

    def __getitem_segment__(self, index, pixel_start: int, target: int, memory_start=None):
        """按指定 (pixel_start, target) 片段加载样本, 跳过内部随机采样.

        与 __getitem__ 的差异:
            * 不再随机选择 pixel_start_choice / target_choice; 直接用调用方
              传入的值. 训练时用于"特殊片段精确混入" — sample_mixed_episodes
              得到的 segment (s, t) 经此入口加载, 保证模型看到的真的是离线
              扫描判定为掉头 / 大角度拐弯的那个片段.
            * memory_start: 默认为 None, 此时与原逻辑一致, 在 [pixel_start, target)
              内随机采样. 也可显式传入 (用于完全确定性回放, 例如调试).
            * use_data_filter 与 max_filter_sample_tries 在此入口被忽略 —
              segment 的连续性由离线扫描保证, 在线再做过滤会破坏混入比例.
              但如果传入的 (pixel_start, target) 越界或非法, 仍会回退到
              安全的默认值, 避免直接崩溃.

        Args:
            index: episode 索引 (dataset_repeat 后的 idx, 与 __getitem__ 一致).
            pixel_start: 起点帧, [0, trajectory_length // 2) 内的整数.
            target: 终点帧, (pixel_start, trajectory_length) 内的整数.
            memory_start: 可选, 若为 None 则在 [pixel_start, target) 随机选;
                也可直接指定.

        Returns:
            与 __getitem__ 相同结构的 18 元组.
        """
        return self._getitem_impl(
            index,
            forced_pixel_start=int(pixel_start),
            forced_target=int(target),
            forced_memory_start=None if memory_start is None else int(memory_start),
        )

    def __getitem__(self, index):
        # Wrap the entire sample construction in a safety net.
        # Corrupt episodes (e.g. missing video columns, out-of-bounds
        # episode indices) raise IndexError at various depths —
        # process_data_parquet, process_memory, load_image, etc.
        # Catching here and resampling guarantees the caller always
        # receives a valid, consistent sample.
        try:
            return self._getitem_impl(index)
        except IndexError:
            return self._skip_corrupt_sample(index, "__getitem__")

    def _getitem_impl(
        self,
        index,
        forced_pixel_start=None,
        forced_target=None,
        forced_memory_start=None,
    ):
        import os
        import time

        if self._last_time is None:
            self._last_time = time.time()
        start_time = time.time()

        (
            camera_intrinsic,
            camera_extrinsic,
            trajectory_extrinsics,
            trajectory_length,
        ) = self.process_data_parquet(index)

        # Degenerate episodes with ≤ 1 frame cannot produce valid
        # (pixel_start, target, memory_start) triplets — every
        # np.random.randint call would have an empty interval.
        if trajectory_length <= 1:
            raise IndexError(
                f"trajectory_length={trajectory_length} for index {index} "
                f"(need at least 2 frames)"
            )

        # Synchronise rgb/depth path lists with the actual trajectory length
        # returned by the parquet read.  Episodes metadata (dataset_from/to)
        # can diverge from the real row count when the episodes parquet was
        # rebuilt by _ensure_episodes_parquet with a stale offset or when the
        # data files are partial.  Clipping here prevents IndexError in
        # process_memory / process_pixel_goal.
        rgb_paths = self.trajectory_rgb_path[index]
        depth_paths = self.trajectory_depth_path[index]
        if len(rgb_paths) != trajectory_length:
            ds_idx, ep_idx, _, _ = self._index_to_ep[index]
            rgb_paths = [
                ("rgb", ds_idx, ep_idx, t) for t in range(trajectory_length)
            ]
            depth_paths = [
                ("depth", ds_idx, ep_idx, t) for t in range(trajectory_length)
            ]

        trajectory_base_extrinsic = camera_extrinsic
        trajectory_obstacle_points, trajectory_inflation_points = self.process_obstacle_points(index)

        if self.random_digit:
            memory_digit = np.random.randint(2, 8)
            pred_digit = memory_digit
        else:
            memory_digit = self.sample_interval
            pred_digit = self.sample_interval

        # ----- 若调用方显式指定了 segment (segment-level GRPO 混入), 直接采用 -----
        # 这里仍要做一次最小越界保护, 防止索引中的 (s, t) 与当前数据集
        # trajectory_length 不一致 (例如重扫前后数据集发生变化).
        force_segment = forced_pixel_start is not None and forced_target is not None
        if force_segment:
            pixel_start_choice = int(forced_pixel_start)
            target_choice = int(forced_target)
            # 钳制到 [0, trajectory_length - 1]
            pixel_start_choice = max(0, min(pixel_start_choice, trajectory_length - 2))
            target_choice = max(pixel_start_choice + 1, min(target_choice, trajectory_length - 1))
            if forced_memory_start is not None:
                memory_start_choice = int(forced_memory_start)
                memory_start_choice = max(pixel_start_choice, min(memory_start_choice, target_choice - 1))
            else:
                # np.random.randint 要求 lo < hi; 若钳制后区间为空, 取中点
                if target_choice > pixel_start_choice:
                    memory_start_choice = int(np.random.randint(pixel_start_choice, target_choice))
                else:
                    memory_start_choice = pixel_start_choice
        elif self.use_data_filter:
            bad_transition, current_valid = self._transition_quality_masks(trajectory_extrinsics)

            sampled = False
            pixel_start_choice, target_choice, memory_start_choice = 0, min(1, trajectory_length - 1), 0
            for _ in range(self.max_filter_sample_tries):
                if self.prior_sample:
                    pixel_start_choice, target_choice = self.rank_steps(
                        trajectory_extrinsics,
                        trajectory_obstacle_points,
                        pred_digit=pred_digit,
                    )
                    if target_choice <= pixel_start_choice:
                        continue
                    memory_start_choice = np.random.randint(pixel_start_choice, target_choice)
                else:
                    pixel_start_choice = np.random.randint(0, max(1, trajectory_length // 2))
                    target_choice = self._sample_target_by_segments(pixel_start_choice, trajectory_length - 1)
                    if target_choice <= pixel_start_choice:
                        continue
                    memory_start_choice = np.random.randint(pixel_start_choice, target_choice)

                if not current_valid[pixel_start_choice] or not current_valid[memory_start_choice] or not current_valid[target_choice]:
                    continue

                if not self._window_continuity_mask(
                    bad_transition,
                    np.array([pixel_start_choice], dtype=np.int64),
                    np.array([target_choice], dtype=np.int64),
                )[0]:
                    continue

                memory_lo = max(0, memory_start_choice - (self.memory_size - 1) * memory_digit)
                if not self._window_continuity_mask(
                    bad_transition,
                    np.array([memory_lo], dtype=np.int64),
                    np.array([memory_start_choice], dtype=np.int64),
                )[0]:
                    continue

                sampled = True
                break

            if not sampled:
                clean_idx = np.where(current_valid)[0]
                if clean_idx.shape[0] >= 2:
                    pixel_start_choice = int(clean_idx[0])
                    target_choice = int(clean_idx[-1])
                    if target_choice <= pixel_start_choice:
                        target_choice = min(trajectory_length - 1, pixel_start_choice + 1)
                    memory_start_choice = pixel_start_choice
                else:
                    pixel_start_choice = 0
                    target_choice = min(trajectory_length - 1, 1)
                    if target_choice <= pixel_start_choice:
                        raise IndexError(
                            f"degenerate trajectory (len={trajectory_length}) "
                            f"has no valid (start, target) pair"
                        )
                    memory_start_choice = 0
        else:
            if self.prior_sample:
                pixel_start_choice, target_choice = self.rank_steps(
                    trajectory_extrinsics,
                    trajectory_obstacle_points,
                    pred_digit=pred_digit,
                )
                if target_choice <= pixel_start_choice:
                    target_choice = min(trajectory_length - 1, pixel_start_choice + 1)
                memory_start_choice = (
                    int(np.random.randint(pixel_start_choice, target_choice))
                    if target_choice > pixel_start_choice
                    else pixel_start_choice
                )
            else:
                pixel_start_choice = np.random.randint(0, max(1, trajectory_length // 2))
                target_choice = self._sample_target_by_segments(pixel_start_choice, trajectory_length - 1)
                if target_choice <= pixel_start_choice:
                    target_choice = min(trajectory_length - 1, pixel_start_choice + 1)
                memory_start_choice = (
                    int(np.random.randint(pixel_start_choice, target_choice))
                    if target_choice > pixel_start_choice
                    else pixel_start_choice
                )

        memory_images, depth_image, memory_index = self.process_memory(
            rgb_paths,
            depth_paths,
            memory_start_choice,
            memory_digit=memory_digit,
        )
        (
            target_local_points,
            augment_local_points,
            target_world_points,
            augment_world_points,
            action_indexes,
        ) = self.process_actions(
            trajectory_extrinsics, trajectory_base_extrinsic, memory_start_choice, target_choice, pred_digit=pred_digit
        )

        init_vector = target_local_points[1] - target_local_points[0]
        self._last_sample_debug = {
            "index": int(index),
            "init_vector": np.asarray(init_vector, dtype=np.float32).copy(),
            "base_heading_angle": float(np.arctan2(init_vector[1], init_vector[0])),
            "memory_start_choice": int(memory_start_choice),
            "target_choice": int(target_choice),
        }
        target_xyt_actions = self.xyz_to_xyt(target_local_points, init_vector)
        augment_xyt_actions = self.xyz_to_xyt(augment_local_points, init_vector)
        pred_actions = target_xyt_actions[action_indexes]
        augment_actions = augment_xyt_actions[action_indexes]
        if trajectory_obstacle_points.shape[0] != 0:
            pred_distance = (
                np.abs(target_world_points[:, np.newaxis, 0:2] - trajectory_obstacle_points[np.newaxis, :, 0:2])
                .sum(axis=-1)
                .min(axis=-1)
            )
            augment_distance = (
                np.abs(augment_world_points[:, np.newaxis, 0:2] - trajectory_obstacle_points[np.newaxis, :, 0:2])
                .sum(axis=-1)
                .min(axis=-1)
            )
            pred_critic = (
                -5.0 * (pred_distance[action_indexes[:-1]] < 0.1).mean()
                + 0.5 * (pred_distance[action_indexes][1:] - pred_distance[action_indexes][:-1]).sum()
            )
            augment_critic = (
                -5.0 * (augment_distance[action_indexes[:-1]] < 0.1).mean()
                + 0.5 * (augment_distance[action_indexes][1:] - augment_distance[action_indexes][:-1]).sum()
            )
        else:
            pred_distance = np.ones(pred_actions.shape[0], dtype=np.float32)
            augment_distance = np.ones(pred_actions.shape[0], dtype=np.float32)
            pred_critic = 2.0
            augment_critic = 2.0

        point_goal = target_xyt_actions[-1]
        image_goal = np.concatenate(
            (
                self.process_image(rgb_paths[target_choice]),
                self.process_image(rgb_paths[memory_start_choice]),
            ),
            axis=-1,
        )

        pixel_target_local_points, _, _, _, _ = self.process_actions(
            trajectory_extrinsics, trajectory_base_extrinsic, pixel_start_choice, target_choice, pred_digit=pred_digit
        )
        pixel_init_vector = pixel_target_local_points[1] - pixel_target_local_points[0]
        pixel_xyt_actions = self.xyz_to_xyt(pixel_target_local_points, pixel_init_vector)
        pixel_goal, pixel_flag = self.process_pixel_goal(
            rgb_paths[pixel_start_choice],
            pixel_xyt_actions[-1],
            camera_intrinsic,
            trajectory_base_extrinsic,
        )
        if self.pixel_channel == 7:
            pixel_goal = np.concatenate((pixel_goal, memory_images[-1]), axis=-1)

        pred_actions = (pred_actions[1:] - pred_actions[:-1]) * 4.0
        augment_actions = (augment_actions[1:] - augment_actions[:-1]) * 4.0

        obstacle_local_points = self._obstacles_to_local(
            trajectory_obstacle_points,
            trajectory_extrinsics[memory_start_choice],
            trajectory_base_extrinsic,
        )

        inflation_local_points = self._obstacles_to_local(
            trajectory_inflation_points,
            trajectory_extrinsics[memory_start_choice],
            trajectory_base_extrinsic,
        )

        world_obstacle_points = trajectory_obstacle_points.astype(np.float32)
        world_inflation_points = trajectory_inflation_points.astype(np.float32)
        base_frame_extrinsic = trajectory_extrinsics[memory_start_choice].astype(np.float32)
        base_extrinsic = trajectory_base_extrinsic.astype(np.float32)

        pred_actions = np.pad(
            pred_actions,
            ((0, 0), (0, self.action_dim - pred_actions.shape[-1])),
            mode='constant',
            constant_values=(0, 0),
        )
        augment_actions = np.pad(
            augment_actions,
            ((0, 0), (0, self.action_dim - augment_actions.shape[-1])),
            mode='constant',
            constant_values=(0, 0),
        )

        end_time = time.time()
        self.item_cnt += 1
        self.batch_time_sum += end_time - start_time
        if self.item_cnt % self.batch_size == 0:
            avg_time = self.batch_time_sum / self.batch_size
            print(
                f'__getitem__ pid={os.getpid()}, avg_time(last {self.batch_size})={avg_time:.2f}s, cnt={self.item_cnt}'
            )
            self.batch_time_sum = 0.0
        point_goal = torch.tensor(point_goal, dtype=torch.float32)
        image_goal = torch.tensor(image_goal, dtype=torch.float32)
        pixel_goal = torch.tensor(pixel_goal, dtype=torch.float32)
        memory_images = torch.tensor(memory_images, dtype=torch.float32)
        depth_image = torch.tensor(depth_image, dtype=torch.float32)
        pred_actions = torch.tensor(pred_actions, dtype=torch.float32)
        augment_actions = torch.tensor(augment_actions, dtype=torch.float32)
        pred_critic = torch.tensor(pred_critic, dtype=torch.float32)
        augment_critic = torch.tensor(augment_critic, dtype=torch.float32)
        obstacle_local_points = torch.tensor(obstacle_local_points, dtype=torch.float32)
        inflation_local_points = torch.tensor(inflation_local_points, dtype=torch.float32)
        world_obstacle_points = torch.tensor(world_obstacle_points, dtype=torch.float32)
        world_inflation_points = torch.tensor(world_inflation_points, dtype=torch.float32)
        base_frame_extrinsic = torch.tensor(base_frame_extrinsic, dtype=torch.float32)
        base_extrinsic = torch.tensor(base_extrinsic, dtype=torch.float32)
        camera_intrinsic_tensor = torch.tensor(camera_intrinsic, dtype=torch.float32)
        camera_extrinsic_tensor = torch.tensor(
            trajectory_extrinsics[memory_start_choice], dtype=torch.float32
        )
        return (
            point_goal,
            image_goal,
            pixel_goal,
            memory_images,
            depth_image,
            pred_actions,
            augment_actions,
            pred_critic,
            augment_critic,
            float(pixel_flag),
            obstacle_local_points,
            world_obstacle_points,
            base_frame_extrinsic,
            base_extrinsic,
            inflation_local_points,
            world_inflation_points,
            camera_intrinsic_tensor,
            camera_extrinsic_tensor,
        )
