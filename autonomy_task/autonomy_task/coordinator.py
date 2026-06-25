"""Multi-robot collection coordinator."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Callable

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

if TYPE_CHECKING:
    from autonomy_msgs.msg import Graph

from autonomy_task.config import TaskConfig, load_config, load_waypoints, resolve_graph_topic, sample_grid
from autonomy_task.dataset_cleanup import cleanup_dataset_roots
from autonomy_task.graph_source import from_graph, merge_waypoints
from autonomy_task.robot import Phase, Robot
from autonomy_task.robot_context import RobotContext
from autonomy_task.state import State, load, save
from autonomy_task.waypoint import Status, Waypoint
from autonomy_task.waypoint_filter import Filter, Pose, bucket_counts_from_records


class Coordinator:
    """Assign waypoints and persist collection progress."""

    def __init__(
        self,
        node: Node,
        cfg: TaskConfig,
        waypoints: list[Waypoint],
        state: State,
        on_update: Callable[[], None] | None = None,
    ) -> None:
        self._node = node
        self._cfg = cfg
        self._wps = waypoints
        self._state = state
        self._on_update = on_update
        self._grid: OccupancyGrid | None = None
        self._filter = Filter(cfg.thresholds, cfg.filter)
        self._robots = [Robot(node, name, cfg) for name in cfg.robot_names()]
        self._contexts: dict[str, RobotContext] = {
            name: RobotContext(goal_frame=cfg.nav_goal_frame(name))
            for name in cfg.robot_names()
        }
        for robot in self._robots:
            robot.set_goal_frame(self._contexts[robot.namespace].goal_frame)
        self._done = False
        self._graph_ready = cfg.waypoint_source != 'graph'
        self._assign_log_tick = 0
        self._assign_fail_ticks: dict[str, int] = {name: 0 for name in cfg.robot_names()}
        if self._wps:
            self._init_waypoints()

    @classmethod
    def from_config(
        cls,
        node: Node,
        path: str | Path,
        on_update: Callable[[], None] | None = None,
        *,
        num_robots: int = 0,
        graph_topic: str = '',
    ) -> Coordinator:
        cfg = load_config(path)
        if num_robots > 0:
            cfg.num_robots = num_robots
        if graph_topic:
            cfg.graph.topic = graph_topic
            cfg.graph.per_robot = False
        cfg.graph.topic = resolve_graph_topic(cfg)
        base = Path(path).parent
        if cfg.waypoint_source == 'graph':
            wps: list[Waypoint] = []
        elif cfg.grid.enabled:
            wps = sample_grid(cfg.grid.__dict__)
        else:
            wp_path = Path(cfg.waypoints_file)
            if not wp_path.is_absolute():
                wp_path = base / wp_path
            wps = load_waypoints(wp_path)
        return cls(node, cfg, wps, load(cfg.state_file), on_update)

    @property
    def graph_ready(self) -> bool:
        return self._graph_ready

    @property
    def config(self) -> TaskConfig:
        return self._cfg

    @property
    def waypoints(self) -> list[Waypoint]:
        return self._wps

    @property
    def robots(self) -> list[Robot]:
        return self._robots

    @property
    def finished(self) -> bool:
        return self._done

    def set_grid(self, msg: OccupancyGrid) -> None:
        self._grid = msg
        if self._wps and not self._cfg.uses_per_robot_maps:
            self._filter.prefilter(self._wps, self._grid)

    def set_robot_grid(self, robot: str, msg: OccupancyGrid) -> None:
        self._context(robot).grid = msg
        if self._wps:
            self._prefilter_per_robot()
            if self._on_update:
                self._on_update()

    def set_graph(self, msg: Graph) -> None:
        self._ingest_graph(self._cfg.robot_names()[0], msg)

    def set_robot_graph(self, robot: str, msg: Graph) -> None:
        self._ingest_graph(robot, msg)

    def wait_robots(self) -> bool:
        t = self._cfg.nav.server_wait_sec
        return all(r.wait_ready(t) for r in self._robots)

    def tick(self) -> None:
        if self._done or not self._graph_ready:
            return
        if not any(r.pose() is not None for r in self._robots):
            return
        for robot in self._robots:
            self._process_robot(robot)
            if robot.phase == Phase.SAVING:
                self._process_robot(robot)
        if self._cfg.thresholds.parallel:
            if not self._target_reached():
                for robot in self._robots:
                    if robot.idle():
                        self._assign(robot)
        elif all(r.idle() for r in self._robots) and not self._target_reached():
            for robot in self._robots:
                if self._assign(robot):
                    break
        # Run again so robots that just entered PRE_RECORD can send nav goals
        # in the same coordinator tick (avoids an extra 0.5s stall).
        for robot in self._robots:
            self._process_robot(robot)
            if robot.phase == Phase.SAVING:
                self._process_robot(robot)
        if self._is_done():
            self._close()
        save(self._cfg.state_file, self._state)
        if self._on_update:
            self._on_update()

    def _process_robot(self, robot: Robot) -> None:
        event = robot.tick()
        if event == 'collected':
            self._collect(robot)
        elif event == 'save_failed':
            self._fail(robot, reason='lerobot_save_failed')
        elif event == 'failed':
            self._fail(robot)
        elif robot.phase == Phase.NAV:
            limit = self._cfg.nav.max_nav_sec + self._cfg.nav.cancel_timeout_sec + 10.0
            if limit > 0.0 and robot.nav_elapsed_sec() > limit:
                robot.force_fail_nav('watchdog')
                if robot.phase == Phase.SAVING:
                    self._process_robot(robot)

    def _target_reached(self) -> bool:
        target = self._cfg.thresholds.target_episodes
        return target > 0 and len(self._state.collected) >= target

    def _assign_level(self, ns: str) -> int:
        ticks = self._assign_fail_ticks.get(ns, 0)
        t = self._cfg.thresholds
        hz = self._cfg.tick_hz
        if ticks >= max(1, int(t.stuck_reposition_sec * hz)):
            return 2
        if ticks >= max(1, int(t.stuck_assign_sec * hz)):
            return 1
        return 0

    def cleanup_datasets(self) -> dict[str, tuple[int, int]]:
        """Remove orphaned LeRobot tmp* dirs under configured dataset roots."""
        if not self._cfg.record.cleanup_tmp_on_start:
            return {}
        roots = self._cfg.dataset_roots(self._cfg.robot_names())
        return cleanup_dataset_roots(roots)

    def shutdown_recording(self) -> None:
        """Best-effort sync stop for any active recordings (e.g. on node exit)."""
        if not rclpy.ok() or not self._node.context.ok:
            return
        for robot in self._robots:
            if robot.recording_active:
                ok, msg = robot.stop_recording_sync()
                level = self._node.get_logger().info if ok else self._node.get_logger().warning
                level(f'{robot.namespace}: shutdown recording ({msg or "stopped"})')

    def trajectory_summary(self) -> dict:
        return self._state.trajectory_summary()

    def summary(self) -> dict:
        return self._state.summary(self._wps)

    def start_recording(self, timeout: float | None = None) -> tuple[bool, str]:
        timeout = timeout if timeout is not None else self._cfg.record.save_timeout_sec
        return self._sync_recording(True, timeout)

    def stop_recording(self, timeout: float | None = None) -> tuple[bool, str]:
        timeout = timeout if timeout is not None else self._cfg.record.save_timeout_sec
        return self._sync_recording(False, timeout)

    def pending_graph_robots(self) -> list[str]:
        if self._cfg.waypoint_source != 'graph' or not self._cfg.graph.per_robot:
            return []
        return [n for n in self._cfg.robot_names() if not self._context(n).graph_ready]

    def pending_costmap_robots(self) -> list[str]:
        if not self._needs_map() or not self._cfg.uses_per_robot_maps:
            return []
        if self._cfg.waypoint_source == 'graph' and not self._cfg.filter.enforce_map_bounds:
            return []
        return [n for n in self._cfg.robot_names() if self._context(n).grid is None]

    def map_topics(self) -> list[str]:
        if not self._needs_map():
            return []
        return [self._cfg.costmap_topic_for(n) for n in self._cfg.robot_names()]

    def graph_topics(self) -> list[str]:
        if self._cfg.waypoint_source != 'graph':
            return []
        if self._cfg.graph.per_robot:
            return [self._cfg.graph_topic_for(n) for n in self._cfg.robot_names()]
        return [self._cfg.graph.topic]

    def _context(self, robot: str) -> RobotContext:
        return self._contexts.setdefault(
            robot, RobotContext(goal_frame=self._cfg.nav_goal_frame(robot)))

    def _needs_map(self) -> bool:
        return self._cfg.filter.require_navigable or self._cfg.filter.enforce_map_bounds

    def _grid_for(self, robot: str) -> OccupancyGrid | None:
        if self._cfg.uses_per_robot_maps:
            return self._context(robot).grid
        return self._grid

    def _graph_ids_for(self, robot: str) -> set[str] | None:
        if self._cfg.waypoint_source != 'graph':
            return None
        if self._cfg.graph.per_robot:
            return self._context(robot).graph_ids()
        ctx = self._context(self._cfg.robot_names()[0])
        return ctx.graph_ids() if ctx.graph_ready else None

    def _ingest_graph(self, robot: str, msg: Graph) -> None:
        new_wps = from_graph(msg, self._cfg.graph.max_nodes)
        ctx = self._context(robot)
        new_ids = {wp.id for wp in new_wps}
        if ctx.graph_ready and ctx.graph_waypoint_ids == new_ids:
            return
        prev_count = len(ctx.graph_waypoint_ids)
        ctx.graph_waypoint_ids = new_ids
        ctx.graph_ready = True
        ctx.goal_frame = self._cfg.nav_goal_frame(robot)
        for r in self._robots:
            if r.namespace == robot:
                r.set_goal_frame(ctx.goal_frame)
        if self._graph_ready and not self._cfg.graph.per_robot and len(new_wps) == len(self._wps):
            return
        self._wps = merge_waypoints(self._wps, new_wps)
        self._init_waypoints()
        self._refresh_graph_ready()
        if len(ctx.graph_waypoint_ids) != prev_count:
            topic = self._cfg.graph_topic_for(robot)
            self._node.get_logger().info(
                f'{robot} graph: {len(ctx.graph_waypoint_ids)} nodes on {topic} '
                f'(pool={len(self._wps)})')
        if self._graph_ready and prev_count != len(ctx.graph_waypoint_ids):
            self._log_wp_stats('after graph')
        if self._on_update:
            self._on_update()

    def _refresh_graph_ready(self) -> None:
        if self._cfg.waypoint_source != 'graph':
            self._graph_ready = True
            return
        names = self._cfg.robot_names()
        if self._cfg.graph.per_robot:
            self._graph_ready = bool(names) and all(self._context(n).graph_ready for n in names)
        else:
            self._graph_ready = any(self._context(n).graph_ready for n in names)

    def _init_waypoints(self) -> None:
        if self._cfg.waypoint_source != 'graph':
            self._filter.dedupe(self._wps)
        collected = set() if self._cfg.thresholds.allow_revisit else self._state.collected_ids
        self._filter.apply_state(self._wps, collected, self._state.failed, self._state.skipped)
        if self._cfg.uses_per_robot_maps:
            self._prefilter_per_robot()
        else:
            self._filter.prefilter(self._wps, self._grid)

    def _prefilter_per_robot(self) -> None:
        if not (self._cfg.filter.enforce_map_bounds or self._cfg.filter.require_navigable):
            return
        for name in self._cfg.robot_names():
            ctx = self._context(name)
            if ctx.grid is None:
                continue
            only_ids = ctx.graph_ids() if self._cfg.waypoint_source == 'graph' else None
            self._filter.prefilter(self._wps, ctx.grid, only_ids=only_ids)

    def _assigned(self, extra: set[str] | None = None) -> set[str]:
        ids = {r.active.id for r in self._robots if r.active}
        if extra:
            ids |= extra
        return ids

    def _peers(self, ns: str) -> tuple[list[Pose], list[Waypoint]]:
        poses: list[Pose] = []
        targets: list[Waypoint] = []
        for r in self._robots:
            if r.namespace == ns:
                continue
            p = r.pose()
            if p is not None:
                poses.append(p)
            if r.active is not None:
                targets.append(r.active)
        return poses, targets

    def _spacing_ref(self) -> list[Waypoint]:
        t = self._cfg.thresholds
        if t.allow_revisit:
            if not t.spacing_vs_collected:
                return []
            return [
                Waypoint(id=r.waypoint_id, x=r.x, y=r.y, yaw=r.yaw)
                for r in self._state.collected
            ]
        return [wp for wp in self._wps if wp.status == Status.COLLECTED]

    def _pending(self) -> list[Waypoint]:
        return [wp for wp in self._wps if wp.status == Status.PENDING]

    def _can_assign_any(self) -> bool:
        for wp in self._pending():
            for r in self._robots:
                peer_poses, peer_targets = self._peers(r.namespace)
                if self._filter.can_assign(
                    wp, r.pose(), self._spacing_ref(), self._assigned(),
                    self._grid_for(r.namespace),
                    valid_graph_ids=self._graph_ids_for(r.namespace),
                    peer_poses=peer_poses, peer_targets=peer_targets,
                )[0]:
                    return True
        return False

    def _assign(self, robot: Robot, *, exclude: set[str] | None = None) -> bool:
        if not robot.idle() or self._target_reached():
            return False
        if robot.pose() is None:
            return False
        ns = robot.namespace
        peer_poses, peer_targets = self._peers(ns)
        level = self._assign_level(ns)
        relax_peer = level >= 1
        bucket_counts = bucket_counts_from_records(
            self._state.collected, self._cfg.thresholds.distance_buckets)
        if level >= 2:
            wp, reason = self._filter.select_reposition(
                self._wps,
                robot.pose(),
                self._spacing_ref(),
                self._assigned(exclude),
                self._grid_for(ns),
                valid_graph_ids=self._graph_ids_for(ns),
                peer_poses=peer_poses,
                peer_targets=peer_targets,
            )
        else:
            wp, reason = self._filter.select_with_reason(
                self._wps,
                robot.pose(),
                self._spacing_ref(),
                self._assigned(exclude),
                self._grid_for(ns),
                valid_graph_ids=self._graph_ids_for(ns),
                peer_poses=peer_poses,
                peer_targets=peer_targets,
                bucket_counts=bucket_counts,
                relax_peer=relax_peer,
            )
        if wp is None:
            self._assign_fail_ticks[ns] = self._assign_fail_ticks.get(ns, 0) + 1
            self._assign_log_tick += 1
            if self._assign_log_tick % max(1, int(self._cfg.tick_hz * 5)) == 1:
                pose = robot.pose()
                pose_s = f'({pose.x:.2f},{pose.y:.2f})' if pose is not None else 'none'
                level_s = ('reposition' if level >= 2
                           else 'relax_peer' if level >= 1 else 'normal')
                self._node.get_logger().info(
                    f'{ns}: no waypoint ({reason}, mode={level_s}), pose={pose_s}, '
                    f'graph_nodes={len(self._graph_ids_for(ns) or ())}, '
                    f'costmap={"yes" if self._grid_for(ns) else "no"}')
            return False
        self._assign_fail_ticks[ns] = 0
        if reason in ('bucket_fallback', 'reposition') or relax_peer:
            self._node.get_logger().info(
                f'{ns}: assign via {reason or "relax_peer"} -> {wp.id}')
        grid = self._grid_for(ns)
        if grid is not None and self._cfg.filter.enforce_map_bounds:
            clamped = self._filter.clamp_to_map(wp.x, wp.y, grid)
            if clamped is None:
                wp.status = Status.SKIPPED
                wp.skip_reason = 'outside_map'
                self._node.get_logger().warning(
                    f'{ns}: skip {wp.id} ({wp.x:.2f}, {wp.y:.2f}) outside map')
                return False
            wp.x, wp.y = clamped
        if not robot.start(wp):
            return False
        frame = robot.goal_frame()
        self._node.get_logger().info(
            f'assign {ns}: {wp.id} -> ({wp.x:.2f}, {wp.y:.2f}) frame={frame}')
        return True

    def _collect(self, robot: Robot) -> None:
        wp = robot.done
        if wp is None:
            return
        stats = robot.trajectory_stats()
        target = self._cfg.thresholds.target_episodes
        collected_n = len(self._state.collected) + 1
        if self._cfg.thresholds.allow_revisit and (target <= 0 or collected_n < target):
            wp.status = Status.PENDING
            wp.collector = ''
        else:
            wp.status = Status.COLLECTED
            wp.collector = robot.namespace
        wp.robot = ''
        wp.skip_reason = ''
        self._state.add_collected(
            wp, robot.namespace,
            path_length_m=stats['path_length_m'],
            straight_line_m=stats['straight_line_m'],
            duration_sec=stats['duration_sec'],
        )
        robot.done = None
        self._node.get_logger().info(
            f'{robot.namespace}: collected {wp.id} '
            f'path={stats["path_length_m"]:.2f}m '
            f'straight={stats["straight_line_m"]:.2f}m '
            f'duration={stats["duration_sec"]:.1f}s')
        if target > 0:
            self._node.get_logger().info(f'progress: {collected_n}/{target} episodes')
        if self._cfg.thresholds.parallel and not self._target_reached():
            self._assign(robot)

    def _fail(self, robot: Robot, *, reason: str = '') -> None:
        wp = robot.done
        if wp is None:
            if not self._target_reached():
                self._assign(robot)
            return
        wp.attempts += 1
        wp.robot = ''
        exclude = {wp.id}
        t = self._cfg.thresholds
        if reason:
            wp.skip_reason = reason
        if t.allow_revisit and wp.attempts <= t.max_retries:
            wp.status = Status.PENDING
            if not reason:
                wp.skip_reason = ''
            label = reason or 'nav failed'
            self._node.get_logger().warning(
                f'{robot.namespace}: {label} {wp.id} '
                f'(attempt {wp.attempts}/{t.max_retries}), reassigning')
        else:
            skip = reason or 'nav_unreachable'
            grid = self._grid_for(robot.namespace)
            if (
                skip == 'nav_unreachable'
                and grid is not None
                and self._cfg.filter.enforce_map_bounds
                and not self._filter.is_planner_reachable(wp.x, wp.y, grid)
            ):
                skip = 'planner_unreachable'
            self._state.add_skipped(wp.id, skip)
            wp.status = Status.SKIPPED
            wp.skip_reason = skip
            self._node.get_logger().warning(
                f'{robot.namespace}: skip {wp.id} ({skip}), reassigning next')
        robot.done = None
        if not self._target_reached():
            self._assign(robot, exclude=exclude)

    def _is_done(self) -> bool:
        if any(not r.idle() for r in self._robots):
            return False
        if not any(r.pose() is not None for r in self._robots):
            return False
        if self._target_reached():
            return True
        if self._cfg.thresholds.target_episodes > 0 and self._cfg.thresholds.allow_revisit:
            return False
        pending = self._pending()
        if not pending:
            return True
        return not self._can_assign_any()

    def _close(self) -> None:
        self._done = True
        pending = len(self._pending())
        target = self._cfg.thresholds.target_episodes
        collected_n = len(self._state.collected)
        if target > 0 and collected_n >= target:
            self._log_wp_stats('target reached')
            self._node.get_logger().info(
                f'collection target reached: {collected_n}/{target} episodes')
        elif pending > 0:
            self._log_exhausted(pending)
        else:
            self._log_wp_stats('collection complete')
        traj = self._state.trajectory_summary()
        self._node.get_logger().info(
            f'trajectories: {traj["episode_count"]} episodes, '
            f'total_path={traj["total_path_m"]:.2f}m, '
            f'total_duration={traj["total_duration_sec"]:.1f}s')
        for robot, rs in traj.get('per_robot', {}).items():
            self._node.get_logger().info(
                f'  {robot}: {rs["count"]} episodes, '
                f'path={rs["total_path_m"]:.2f}m, '
                f'duration={rs["total_duration_sec"]:.1f}s')
        if pending > 0:
            self._node.get_logger().warn(
                f'collection stopped with {pending} pending '
                '(exhausted — not all graph nodes collected)')
        else:
            self._node.get_logger().info('collection complete')

    def _log_wp_stats(self, prefix: str) -> None:
        counts = {s.value: 0 for s in Status}
        for wp in self._wps:
            counts[wp.status.value] += 1
        self._node.get_logger().info(
            f'{prefix}: pending={counts["pending"]} skipped={counts["skipped"]} '
            f'collected={counts["collected"]} failed={counts["failed"]} '
            f'in_progress={counts["in_progress"]}')

    def _log_exhausted(self, pending: int) -> None:
        reasons: dict[str, int] = {}
        for wp in self._pending():
            for robot in self._robots:
                ns = robot.namespace
                peer_poses, peer_targets = self._peers(ns)
                _ok, reason = self._filter.can_assign(
                    wp, robot.pose(), self._spacing_ref(), self._assigned(),
                    self._grid_for(ns), valid_graph_ids=self._graph_ids_for(ns),
                    peer_poses=peer_poses, peer_targets=peer_targets,
                )
                key = reason or 'eligible'
                reasons[key] = reasons.get(key, 0) + 1
        top = sorted(reasons.items(), key=lambda x: -x[1])[:5]
        detail = ', '.join(f'{k}={v}' for k, v in top)
        self._log_wp_stats('collection exhausted')
        self._node.get_logger().warn(
            f'{pending} pending waypoints but none assignable '
            f'(try lowering min_spacing_m / min_distance_m). top reasons: {detail}')

    def _sync_recording(self, on: bool, timeout: float) -> tuple[bool, str]:
        if not self._cfg.record.enabled:
            return False, 'recording disabled in config'
        lines: list[str] = []
        ok_all = True
        for robot in self._robots:
            if on:
                ok, msg = robot.set_recording_sync(True, timeout=timeout)
            elif robot.recording_active:
                ok, msg = robot.stop_recording_sync()
            else:
                ok, msg = True, 'not recording'
            ok_all = ok_all and ok
            level = self._node.get_logger().info if ok else self._node.get_logger().warning
            level(f'{robot.namespace}: {msg}')
            lines.append(f'{robot.namespace}: {msg}')
        return ok_all, '; '.join(lines)
