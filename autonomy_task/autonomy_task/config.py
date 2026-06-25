"""Task configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from autonomy_task.waypoint import Waypoint


@dataclass
class NavConfig:
    action_timeout_sec: float = 300.0
    server_wait_sec: float = 60.0
    goal_tolerance_m: float = 0.35
    # Cancel navigate_to_pose when progress toward the goal stalls (Nav2 BT may
    # keep recovering follow_path without failing the top-level action).
    stall_sec: float = 10.0
    stall_move_m: float = 0.3
    stall_goal_progress_m: float = 0.25
    # Hard cap per waypoint attempt; fail and reassign even if Nav2 BT keeps recovering.
    max_nav_sec: float = 45.0
    # Force fail if cancel_goal does not complete (avoids blocking reassignment).
    cancel_timeout_sec: float = 5.0


@dataclass
class Thresholds:
    min_distance_m: float = 4.0
    max_distance_m: float = 50.0
    min_spacing_m: float = 4.0
    max_spacing_m: float = 0.0
    dedupe_radius_m: float = 2.5
    assignment_strategy: str = 'farthest'  # nearest | farthest | dispersed | bucket
    min_peer_spacing_m: float = 3.0
    arrival_radius_m: float = 0.2
    record_before_sec: float = 0.5
    record_after_sec: float = 1.0
    max_retries: int = 2
    parallel: bool = True
    target_episodes: int = 0
    allow_revisit: bool = False
    spacing_vs_collected: bool = True
    distance_buckets: list[tuple[float, float]] = field(default_factory=list)
    # After this many seconds without a waypoint, relax peer spacing so idle robots
    # can still navigate (e.g. when the priority distance bucket has no local nodes).
    stuck_assign_sec: float = 15.0
    # After this many seconds still unassigned, send the robot to the farthest
    # reachable waypoint (reposition) to unlock new distance buckets.
    stuck_reposition_sec: float = 30.0

    def per_bucket_target(self) -> float:
        n = len(self.distance_buckets)
        if n == 0 or self.target_episodes <= 0:
            return 0.0
        return self.target_episodes / n


@dataclass
class RecordConfig:
    enabled: bool = True
    bridge: str = 'lerobot_bridge_node'
    # Wait for lerobot_bridge set_recording(false) + save_episode (video encode).
    save_timeout_sec: float = 120.0
    cleanup_tmp_on_start: bool = True


# Defaults aligned with autonomy_lerobot/config/lerobot_collection.yaml; override via launch.
_DEFAULT_DATASET_ROOT = '/workspace/autonomy/data/lerobot/collection'
_DEFAULT_DATASET_REPO_ID = 'local/habitat_collection'


@dataclass
class FilterConfig:
    require_navigable: bool = False
    enforce_map_bounds: bool = True
    map_margin_m: float = 0.5
    # Extra inset from map edges so Nav2 GridBased tolerance search stays in-bounds.
    planner_margin_m: float = 1.0
    map_topic: str = 'map'
    costmap_topic: str = 'global_costmap/costmap'
    occupied_threshold: int = 50
    free_threshold: int = 0
    use_global_costmap: bool = True


@dataclass
class GraphConfig:
    topic: str = '/robot1/habitat/graph'
    topic_suffix: str = 'habitat/graph'
    per_robot: bool = True
    wait_sec: float = 60.0
    max_nodes: int = 0
    use_frame: bool = True


_LEGACY_GRAPH_TOPICS = frozenset({'/habitat/graph', 'habitat/graph'})


def resolve_graph_topic(cfg: TaskConfig) -> str:
    """Map legacy global graph topic to namespaced robot1 topic in multi-robot mode."""
    topic = cfg.graph.topic.strip()
    if not topic:
        topic = 'habitat/graph'
    if not topic.startswith('/'):
        topic = f'/{topic}'
    if topic not in _LEGACY_GRAPH_TOPICS:
        return topic
    names = cfg.robot_names()
    if not names:
        return topic
    return f'/{names[0]}/habitat/graph'


@dataclass
class GridConfig:
    enabled: bool = False
    origin_x: float = 0.0
    origin_y: float = 0.0
    rows: int = 0
    cols: int = 0
    step_m: float = 1.0
    yaw: float = 0.0


@dataclass
class RobotEntry:
    name: str
    enabled: bool = True


@dataclass
class TaskConfig:
    frame_id: str = 'map'
    robot_prefix: str = 'robot'
    num_robots: int = 1
    robots: list[RobotEntry] = field(default_factory=list)
    nav: NavConfig = field(default_factory=NavConfig)
    thresholds: Thresholds = field(default_factory=Thresholds)
    record: RecordConfig = field(default_factory=RecordConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    graph: GraphConfig = field(default_factory=GraphConfig)
    grid: GridConfig = field(default_factory=GridConfig)
    waypoint_source: str = 'file'
    waypoints_file: str = ''
    status_hz: float = 2.0
    tick_hz: float = 2.0
    state_file: str = '/workspace/autonomy/data/collection/state.json'
    dataset_root: str = _DEFAULT_DATASET_ROOT
    dataset_repo_id: str = _DEFAULT_DATASET_REPO_ID
    per_robot_dataset: bool = True

    def dataset_root_for(self, robot: str) -> str:
        root = Path(self.dataset_root).expanduser()
        if self.per_robot_dataset:
            return str(root / robot)
        return str(root)

    def dataset_roots(self, robots: list[str]) -> list[Path]:
        root = Path(self.dataset_root).expanduser()
        if self.per_robot_dataset:
            return [root / name for name in robots]
        return [root]

    def ensure_output_dirs(self, robots: list[str]) -> None:
        Path(self.state_file).expanduser().parent.mkdir(parents=True, exist_ok=True)

    def apply_dataset(
        self,
        *,
        dataset_root: str = '',
        dataset_repo_id: str = '',
        per_robot_dataset: bool | None = None,
    ) -> None:
        if dataset_root:
            self.dataset_root = dataset_root
        if dataset_repo_id:
            self.dataset_repo_id = dataset_repo_id
        if per_robot_dataset is not None:
            self.per_robot_dataset = per_robot_dataset

    def robot_names(self) -> list[str]:
        if self.robots:
            return [r.name for r in self.robots if r.enabled]
        return [f'{self.robot_prefix}{i}' for i in range(1, self.num_robots + 1)]

    def graph_topic_for(self, robot: str) -> str:
        """Resolve graph topic for one robot or the shared legacy topic."""
        if self.graph.per_robot:
            suffix = self.graph.topic_suffix.strip('/')
            return f'/{robot}/{suffix}'
        topic = self.graph.topic.strip()
        if not topic.startswith('/'):
            topic = f'/{topic}'
        return topic

    def costmap_topic_for(self, robot: str) -> str:
        """Nav2 global costmap topic for one robot namespace."""
        if self.filter.use_global_costmap and self.filter.costmap_topic:
            topic = self.filter.costmap_topic.strip('/')
        else:
            topic = self.filter.map_topic.strip('/')
        if topic.startswith('/'):
            return topic
        return f'/{robot}/{topic}'

    def nav_goal_frame(self, robot: str = '') -> str:
        """Frame for NavigateToPose inside a namespaced Nav2 stack (always ``map``)."""
        _ = robot
        fid = self.frame_id.strip('/')
        if not fid:
            return 'map'
        return fid.split('/')[-1] or 'map'

    @property
    def uses_per_robot_maps(self) -> bool:
        return self.graph.per_robot or self.num_robots > 1

    @property
    def marker_frame(self) -> str:
        """RViz marker frame (namespaced map for multi-robot graph collection)."""
        names = self.robot_names()
        if names and self.waypoint_source == 'graph':
            return f'/{names[0]}/map'
        fid = self.frame_id
        if fid.startswith('/'):
            return fid
        if names:
            return f'/{names[0]}/{fid}'
        return fid


def _dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _parse_distance_buckets(raw: Any) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    if not isinstance(raw, list):
        return out
    for item in raw:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            lo, hi = float(item[0]), float(item[1])
            if hi > lo:
                out.append((lo, hi))
    return out


def _thresholds(data: dict[str, Any]) -> Thresholds:
    return Thresholds(
        min_distance_m=float(data.get('min_distance_m', 0.5)),
        max_distance_m=float(data.get('max_distance_m', 20.0)),
        min_spacing_m=float(data.get('min_spacing_m', 1.0)),
        max_spacing_m=float(data.get('max_spacing_m', 0.0)),
        dedupe_radius_m=float(data.get('dedupe_radius_m', 2.5)),
        assignment_strategy=str(data.get('assignment_strategy', 'farthest')),
        min_peer_spacing_m=float(data.get('min_peer_spacing_m', 3.0)),
        arrival_radius_m=float(data.get('arrival_radius_m', 0.2)),
        record_before_sec=float(data.get('record_before_nav_sec', data.get('record_before_sec', 0.5))),
        record_after_sec=float(data.get('record_after_arrival_sec', data.get('record_after_sec', 1.0))),
        max_retries=int(data.get('max_retries', 2)),
        parallel=bool(data.get('parallel', True)),
        target_episodes=int(data.get('target_episodes', 0)),
        allow_revisit=bool(data.get('allow_revisit', False)),
        spacing_vs_collected=bool(data.get('spacing_vs_collected', True)),
        distance_buckets=_parse_distance_buckets(data.get('distance_buckets')),
        stuck_assign_sec=float(data.get('stuck_assign_sec', 15.0)),
        stuck_reposition_sec=float(data.get('stuck_reposition_sec', 30.0)),
    )


def load_config(path: str | Path) -> TaskConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(raw, dict):
        raise ValueError(f'config must be a mapping: {path}')
    task = _dict(raw.get('collection_task', raw))
    nav = _dict(task.get('navigation'))
    filt = _dict(task.get('filter'))
    grid = _dict(task.get('grid_sampling', task.get('grid')))
    graph = _dict(task.get('graph'))
    rec = _dict(task.get('recording'))
    out = _dict(task.get('output'))

    robots: list[RobotEntry] = []
    for entry in task.get('robots', []):
        if isinstance(entry, str):
            robots.append(RobotEntry(name=entry))
        elif isinstance(entry, dict) and 'name' in entry:
            robots.append(RobotEntry(
                name=str(entry['name']),
                enabled=bool(entry.get('enabled', True)),
            ))

    cfg = TaskConfig(
        frame_id=str(task.get('frame_id', 'map')),
        robot_prefix=str(task.get('robot_prefix', 'robot')),
        num_robots=int(task.get('num_robots', 1)),
        robots=robots,
        nav=NavConfig(
            action_timeout_sec=float(nav.get('action_timeout_sec', 300.0)),
            server_wait_sec=float(nav.get('server_wait_sec', 60.0)),
            goal_tolerance_m=float(nav.get('goal_tolerance_m', 0.15)),
            stall_sec=float(nav.get('stall_sec', 10.0)),
            stall_move_m=float(nav.get('stall_move_m', 0.3)),
            stall_goal_progress_m=float(nav.get('stall_goal_progress_m', 0.25)),
            max_nav_sec=float(nav.get('max_nav_sec', 45.0)),
            cancel_timeout_sec=float(nav.get('cancel_timeout_sec', 5.0)),
        ),
        thresholds=_thresholds(_dict(task.get('collection'))),
        record=RecordConfig(
            enabled=bool(rec.get('enabled', True)),
            bridge=str(rec.get('bridge_basename', rec.get('bridge', 'lerobot_bridge_node'))),
            save_timeout_sec=float(rec.get('save_timeout_sec', 120.0)),
            cleanup_tmp_on_start=bool(rec.get('cleanup_tmp_on_start', True)),
        ),
        state_file=str(
            task.get('state_file', out.get('state_file', '/workspace/autonomy/data/collection/state.json'))
        ),
        dataset_root=str(out.get('dataset_root', _DEFAULT_DATASET_ROOT)),
        dataset_repo_id=str(out.get('dataset_repo_id', _DEFAULT_DATASET_REPO_ID)),
        per_robot_dataset=bool(out.get('per_robot_dataset', True)),
        filter=FilterConfig(
            require_navigable=bool(filt.get('require_navigable', False)),
            enforce_map_bounds=bool(filt.get('enforce_map_bounds', True)),
            map_margin_m=float(filt.get('map_margin_m', 0.5)),
            planner_margin_m=float(filt.get('planner_margin_m', 1.0)),
            map_topic=str(filt.get('map_topic', 'map')),
            costmap_topic=str(filt.get('costmap_topic', 'global_costmap/costmap')),
            occupied_threshold=int(filt.get('occupied_threshold', 50)),
            free_threshold=int(filt.get('free_threshold', 0)),
            use_global_costmap=bool(filt.get('use_global_costmap', True)),
        ),
        grid=GridConfig(
            enabled=bool(grid.get('enabled', False)),
            origin_x=float(grid.get('origin_x', 0.0)),
            origin_y=float(grid.get('origin_y', 0.0)),
            rows=int(grid.get('rows', 0)),
            cols=int(grid.get('cols', 0)),
            step_m=float(grid.get('step_m', 1.0)),
            yaw=float(grid.get('yaw', 0.0)),
        ),
        graph=GraphConfig(
            topic=str(graph.get('topic', '/robot1/habitat/graph')),
            topic_suffix=str(graph.get('topic_suffix', 'habitat/graph')),
            per_robot=bool(graph.get('per_robot', True)),
            wait_sec=float(graph.get('wait_sec', 60.0)),
            max_nodes=int(graph.get('max_nodes', 0)),
            use_frame=bool(graph.get('use_frame', True)),
        ),
        waypoint_source=str(task.get('waypoint_source', 'file')),
        waypoints_file=str(task.get('waypoints_file', '')),
        status_hz=float(task.get('status_publish_hz', task.get('status_hz', 2.0))),
        tick_hz=float(task.get('coordinator_tick_hz', task.get('tick_hz', 2.0))),
    )
    cfg.graph.topic = resolve_graph_topic(cfg)
    return cfg


def load_waypoints(path: str | Path) -> list[Waypoint]:
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'waypoints file must be a mapping: {path}')
    if 'grid_sampling' in data or 'grid' in data:
        return sample_grid(_dict(data.get('grid_sampling', data.get('grid'))))
    entries = data.get('waypoints', data.get('poses', []))
    if not isinstance(entries, list):
        raise ValueError(f'waypoints file missing list: {path}')
    out: list[Waypoint] = []
    for i, entry in enumerate(entries):
        if not isinstance(entry, dict):
            continue
        wp_id = str(entry.get('id', f'wp_{i:04d}'))
        out.append(Waypoint(
            id=wp_id,
            x=float(entry['x']),
            y=float(entry['y']),
            yaw=float(entry.get('yaw', 0.0)),
            z=float(entry.get('z', 0.0)),
            label=str(entry.get('label', wp_id)),
        ))
    return out


def sample_grid(grid: dict[str, Any]) -> list[Waypoint]:
    if not grid.get('enabled', True):
        return []
    rows, cols = int(grid.get('rows', 0)), int(grid.get('cols', 0))
    step = float(grid.get('step_m', 1.0))
    ox, oy = float(grid.get('origin_x', 0.0)), float(grid.get('origin_y', 0.0))
    yaw = float(grid.get('yaw', 0.0))
    return [
        Waypoint(
            id=f'grid_{r:03d}_{c:03d}',
            x=ox + c * step,
            y=oy + r * step,
            yaw=yaw,
            label=f'grid_{r:03d}_{c:03d}',
        )
        for r in range(rows)
        for c in range(cols)
    ]
