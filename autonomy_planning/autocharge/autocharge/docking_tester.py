"""Docking test monitor: subscribe metrics, visualize, and persist data."""

from __future__ import annotations

import csv
import os
import math
import random
from datetime import datetime
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped

from autocharge.docking.utils import dock_front_point

try:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    _HAS_MPL = True
except Exception:  # pragma: no cover - optional runtime dependency
    _HAS_MPL = False


class DockingTester(Node):
    """Collects docking metrics and saves CSV/figure artifacts."""

    def __init__(self) -> None:
        super().__init__('docking_tester')
        self._declare_parameters()
        self._load_paths_and_eval_targets()
        self._load_runtime_state()
        self._normalize_random_pose_ranges()
        self._create_interfaces()
        self._log_startup()

    def _declare_parameters(self) -> None:
        self.declare_parameter('metrics_topic', '/dock/test_metrics')
        self.declare_parameter('state_topic', '/dock/state')
        self.declare_parameter('save_dir', '~/dock_test_logs')
        self.declare_parameter('save_interval_s', 2.0)
        self.declare_parameter('plot_enable', True)
        self.declare_parameter('max_tests', 10)
        self.declare_parameter('case_timeout_s', 50.0)
        self.declare_parameter('success_state', 'SUCCESS')
        self.declare_parameter('fail_states', ['ABORT', 'FAIL'])
        self.declare_parameter('auto_stop_when_done', True)
        self.declare_parameter('send_reset_before_test', False)
        self.declare_parameter('reset_topic', '/dock/reset')
        self.declare_parameter('reset_delay_s', 0.3)
        self.declare_parameter('send_start_after_reset', True)
        self.declare_parameter('start_topic', '/dock/start')
        self.declare_parameter('start_delay_s', 0.8)
        self.declare_parameter('reset_after_case', False)
        self.declare_parameter('start_after_case', True)
        self.declare_parameter('inter_case_start_delay_s', 0.8)
        self.declare_parameter('min_case_duration_s', 1.0)
        self.declare_parameter('randomize_pose_before_start', True)
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('random_pose_x_min', -0.2)
        self.declare_parameter('random_pose_x_max', 0.2)
        self.declare_parameter('random_pose_y_min', -0.2)
        self.declare_parameter('random_pose_y_max', 0.2)
        self.declare_parameter('random_pose_yaw_min', -0.4)
        self.declare_parameter('random_pose_yaw_max', 0.4)
        # pose_schedule: random | grid（网格：纵向距离 × 横向偏移 × 重复次数）
        self.declare_parameter('pose_schedule', 'random')
        self.declare_parameter('grid_scenario', '')
        self.declare_parameter('grid_longitudinal_m', 0.5)
        self.declare_parameter(
            'grid_lateral_offsets_m',
            [0.05, -0.05, 0.10, -0.10, 0.15, -0.15, 0.20, -0.20, 0.25, -0.25, 0.30, -0.30],
        )
        self.declare_parameter('grid_repeats_per_offset', 10)
        self.declare_parameter('grid_robot_yaw', 0.0)
        self.declare_parameter('dock_x', 0.5)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_yaw', 3.1415926)
        self.declare_parameter('dock_front_x_offset_m', 0.0)
        self.declare_parameter('dock_front_y_offset_m', 0.0)
        self.declare_parameter('success_contact_x_m', 0.16)
        self.declare_parameter('success_contact_y_m', 0.0)
        # 全部测试结束后：两项距离指标相对目标的平均绝对误差（米）
        self.declare_parameter('target_left_ir_to_dock_right_mid_m', 0.0)
        self.declare_parameter('target_front_contact_to_dock_center_m', 0.0)
        self.declare_parameter('eval_errors_success_rows_only', False)

    def _load_paths_and_eval_targets(self) -> None:
        save_root = os.path.expanduser(str(self.get_parameter('save_dir').value))
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._run_dir = os.path.join(save_root, f'run_{ts}')
        os.makedirs(self._run_dir, exist_ok=True)

        self._csv_path = os.path.join(self._run_dir, 'metrics.csv')
        self._plot_path = os.path.join(self._run_dir, 'metrics.png')
        self._summary_path = os.path.join(self._run_dir, 'summary.csv')
        self._eval_path = os.path.join(self._run_dir, 'evaluation.csv')
        self._eval_rounds_path = os.path.join(self._run_dir, 'evaluation_rounds.csv')
        self._group_summary_path = os.path.join(self._run_dir, 'success_rate_by_offset.csv')
        self._eval_rounds_header_written = False
        self._target_left_ir_m = float(
            self.get_parameter('target_left_ir_to_dock_right_mid_m').value
        )
        self._target_front_m = float(
            self.get_parameter('target_front_contact_to_dock_center_m').value
        )
        self._eval_errors_success_only = bool(
            self.get_parameter('eval_errors_success_rows_only').value
        )

    def _load_runtime_state(self) -> None:
        self._plot_enable = bool(self.get_parameter('plot_enable').value) and _HAS_MPL
        self._latest_state_topic: Optional[str] = None
        self._dirty = False
        self._rows: list[dict[str, str]] = []
        self._t0 = self.get_clock().now()
        self._max_tests = max(1, int(self.get_parameter('max_tests').value))
        self._case_timeout_s = max(1.0, float(self.get_parameter('case_timeout_s').value))
        self._success_state = str(self.get_parameter('success_state').value).strip().upper()
        self._fail_states = {
            str(s).strip().upper() for s in list(self.get_parameter('fail_states').value)
        }
        self._auto_stop_when_done = bool(self.get_parameter('auto_stop_when_done').value)
        self._case_active = False
        self._case_start_t: Optional[float] = None
        self._case_index = 0
        self._finished = False
        self._results: list[dict[str, str]] = []
        self._send_reset_before_test = bool(self.get_parameter('send_reset_before_test').value)
        self._reset_sent = False
        self._send_start_after_reset = bool(self.get_parameter('send_start_after_reset').value)
        self._start_sent = False
        self._reset_after_case = bool(self.get_parameter('reset_after_case').value)
        self._start_after_case = bool(self.get_parameter('start_after_case').value)
        self._inter_case_start_delay_s = max(
            0.0, float(self.get_parameter('inter_case_start_delay_s').value)
        )
        self._min_case_duration_s = max(0.0, float(self.get_parameter('min_case_duration_s').value))
        self._inter_case_start_timer = None
        self._reset_pulse_off_timer = None
        self._await_case_running = True
        self._shutdown_requested = False
        self._shutdown_timer = None
        self._randomize_pose_before_start = bool(
            self.get_parameter('randomize_pose_before_start').value
        )
        self._random_pose_x_min = float(self.get_parameter('random_pose_x_min').value)
        self._random_pose_x_max = float(self.get_parameter('random_pose_x_max').value)
        self._random_pose_y_min = float(self.get_parameter('random_pose_y_min').value)
        self._random_pose_y_max = float(self.get_parameter('random_pose_y_max').value)
        self._random_pose_yaw_min = float(self.get_parameter('random_pose_yaw_min').value)
        self._random_pose_yaw_max = float(self.get_parameter('random_pose_yaw_max').value)
        self._pose_schedule = str(self.get_parameter('pose_schedule').value).strip().lower()
        self._grid_scenario = str(self.get_parameter('grid_scenario').value).strip()
        self._grid_longitudinal_m = float(self.get_parameter('grid_longitudinal_m').value)
        self._grid_lateral_offsets_m = [
            float(v) for v in list(self.get_parameter('grid_lateral_offsets_m').value)
        ]
        self._grid_repeats_per_offset = max(
            1, int(self.get_parameter('grid_repeats_per_offset').value)
        )
        self._grid_robot_yaw = float(self.get_parameter('grid_robot_yaw').value)
        self._dock_x = float(self.get_parameter('dock_x').value)
        self._dock_y = float(self.get_parameter('dock_y').value)
        self._dock_yaw = float(self.get_parameter('dock_yaw').value)
        self._dock_front_x_offset_m = float(self.get_parameter('dock_front_x_offset_m').value)
        self._dock_front_y_offset_m = float(self.get_parameter('dock_front_y_offset_m').value)
        self._success_contact_x_m = float(self.get_parameter('success_contact_x_m').value)
        self._success_contact_y_m = float(self.get_parameter('success_contact_y_m').value)
        self._grid_pose_plan: list[dict[str, Any]] = []
        self._grid_pose_index = 0
        self._case_pose_meta: dict[str, str] = {}
        self._apply_pose_schedule()

    def _apply_pose_schedule(self) -> None:
        if self._pose_schedule != 'grid':
            return
        if not self._grid_lateral_offsets_m:
            self.get_logger().error('grid mode requires non-empty grid_lateral_offsets_m')
            return
        self._grid_pose_plan = self._build_grid_pose_plan()
        planned = len(self._grid_pose_plan)
        self._max_tests = planned
        self.get_logger().info(
            'grid test plan: scenario=%s longitudinal_m=%.3f offsets=%d repeats=%d total_cases=%d'
            % (
                self._grid_scenario or 'unspecified',
                self._grid_longitudinal_m,
                len(self._grid_lateral_offsets_m),
                self._grid_repeats_per_offset,
                planned,
            )
        )

    def _robot_base_pose_for_grid(
        self, longitudinal_m: float, lateral_m: float
    ) -> tuple[float, float, float]:
        """纵向距离：桩面到机器人 base_link 沿对接方向的间距（米）。0 为接触点贴桩面。"""
        dock_front_x, dock_front_y = dock_front_point(
            self._dock_x,
            self._dock_y,
            self._dock_yaw,
            self._dock_front_x_offset_m,
            self._dock_front_y_offset_m,
        )
        yaw = float(self._grid_robot_yaw)
        long_m = max(0.0, float(longitudinal_m))
        lat_m = float(lateral_m)
        if long_m <= 1e-6:
            contact_x = dock_front_x
            contact_y = dock_front_y + lat_m
            c = math.cos(yaw)
            s = math.sin(yaw)
            base_x = contact_x - (c * self._success_contact_x_m - s * self._success_contact_y_m)
            base_y = contact_y - (s * self._success_contact_x_m + c * self._success_contact_y_m)
        else:
            base_x = dock_front_x - long_m
            base_y = dock_front_y + lat_m
        return base_x, base_y, yaw

    def _build_grid_pose_plan(self) -> list[dict[str, Any]]:
        plan: list[dict[str, Any]] = []
        for lateral_m in self._grid_lateral_offsets_m:
            for rep in range(self._grid_repeats_per_offset):
                x, y, yaw = self._robot_base_pose_for_grid(
                    self._grid_longitudinal_m, lateral_m
                )
                plan.append(
                    {
                        'x': x,
                        'y': y,
                        'yaw': yaw,
                        'longitudinal_m': f'{self._grid_longitudinal_m:.4f}',
                        'lateral_offset_m': f'{lateral_m:.4f}',
                        'repeat_index': str(rep + 1),
                        'grid_scenario': self._grid_scenario,
                    }
                )
        return plan

    def _normalize_random_pose_ranges(self) -> None:
        if self._random_pose_x_min > self._random_pose_x_max:
            self._random_pose_x_min, self._random_pose_x_max = (
                self._random_pose_x_max,
                self._random_pose_x_min,
            )
        if self._random_pose_y_min > self._random_pose_y_max:
            self._random_pose_y_min, self._random_pose_y_max = (
                self._random_pose_y_max,
                self._random_pose_y_min,
            )
        if self._random_pose_yaw_min > self._random_pose_yaw_max:
            self._random_pose_yaw_min, self._random_pose_yaw_max = (
                self._random_pose_yaw_max,
                self._random_pose_yaw_min,
            )

    def _create_interfaces(self) -> None:
        metrics_topic = str(self.get_parameter('metrics_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        self.create_subscription(String, metrics_topic, self._cb_metrics, 50)
        self.create_subscription(String, state_topic, self._cb_state, 20)
        self._reset_pub = self.create_publisher(
            Bool, str(self.get_parameter('reset_topic').value), 10
        )
        self._start_pub = self.create_publisher(
            Bool, str(self.get_parameter('start_topic').value), 10
        )
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initialpose_topic').value),
            10,
        )
        save_interval = max(0.5, float(self.get_parameter('save_interval_s').value))
        self.create_timer(save_interval, self._on_timer_save)
        reset_delay = max(0.0, float(self.get_parameter('reset_delay_s').value))
        self.create_timer(reset_delay, self._on_timer_reset_once)
        start_delay = max(0.0, float(self.get_parameter('start_delay_s').value))
        self.create_timer(start_delay, self._on_timer_start_once)

    def _log_startup(self) -> None:
        self.get_logger().info(f'docking_tester started, save_dir={self._run_dir}')
        self.get_logger().info(
            'test plan: max_tests=%d case_timeout_s=%.1f success_state=%s fail_states=%s'
            % (
                self._max_tests,
                self._case_timeout_s,
                self._success_state,
                sorted(self._fail_states),
            )
        )
        if not _HAS_MPL:
            self.get_logger().warning('matplotlib not available, only CSV will be saved')
        if self._pose_schedule == 'grid':
            self.get_logger().info(
                'grid offsets (m): %s'
                % ', '.join(f'{v:+.3f}' for v in self._grid_lateral_offsets_m)
            )

    def _make_bool_msg(self, value: bool) -> Bool:
        msg = Bool()
        msg.data = bool(value)
        return msg

    def _on_timer_reset_once(self) -> None:
        if self._reset_sent or (not self._send_reset_before_test):
            return
        self._publish_reset()
        self._reset_sent = True
        self.get_logger().info('tester pre-task reset published')

    def _on_timer_start_once(self) -> None:
        if self._start_sent or (not self._send_start_after_reset):
            return
        self._publish_start()
        self._start_sent = True
        self.get_logger().info('tester pre-task start published')

    def _publish_reset(self) -> None:
        msg = self._make_bool_msg(True)
        self._reset_pub.publish(msg)
        # Send a short pulse (True -> False) to make reset edge explicit.
        if self._reset_pulse_off_timer is not None:
            self._reset_pulse_off_timer.cancel()
            self._reset_pulse_off_timer = None
        self._reset_pulse_off_timer = self.create_timer(0.05, self._publish_reset_off_once)

    def _publish_reset_off_once(self) -> None:
        if self._reset_pulse_off_timer is not None:
            self._reset_pulse_off_timer.cancel()
            self._reset_pulse_off_timer = None
        msg = self._make_bool_msg(False)
        self._reset_pub.publish(msg)

    def _publish_start(self) -> None:
        self._publish_case_initial_pose()
        msg = self._make_bool_msg(True)
        self._start_pub.publish(msg)
        self._begin_case_on_start()

    def _publish_case_initial_pose(self) -> None:
        if not self._randomize_pose_before_start:
            return
        meta: dict[str, str] = {}
        if self._pose_schedule == 'grid' and self._grid_pose_plan:
            idx = min(self._grid_pose_index, len(self._grid_pose_plan) - 1)
            entry = self._grid_pose_plan[idx]
            x = float(entry['x'])
            y = float(entry['y'])
            yaw = float(entry['yaw'])
            meta = {
                'longitudinal_m': str(entry['longitudinal_m']),
                'lateral_offset_m': str(entry['lateral_offset_m']),
                'repeat_index': str(entry['repeat_index']),
                'grid_scenario': str(entry.get('grid_scenario', '')),
                'start_x_m': f'{x:.6f}',
                'start_y_m': f'{y:.6f}',
                'start_yaw_rad': f'{yaw:.6f}',
            }
            self._grid_pose_index += 1
        else:
            x = random.uniform(self._random_pose_x_min, self._random_pose_x_max)
            y = random.uniform(self._random_pose_y_min, self._random_pose_y_max)
            yaw = random.uniform(self._random_pose_yaw_min, self._random_pose_yaw_max)
            meta = {
                'start_x_m': f'{x:.6f}',
                'start_y_m': f'{y:.6f}',
                'start_yaw_rad': f'{yaw:.6f}',
            }
        half = 0.5 * yaw
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        self._initialpose_pub.publish(msg)
        self._case_pose_meta = meta
        if self._pose_schedule == 'grid' and meta:
            self.get_logger().info(
                'tester grid initialpose: scenario=%s long=%sm lat=%sm rep=%s x=%.3f y=%.3f yaw=%.3f'
                % (
                    meta.get('grid_scenario', ''),
                    meta.get('longitudinal_m', ''),
                    meta.get('lateral_offset_m', ''),
                    meta.get('repeat_index', ''),
                    x,
                    y,
                    yaw,
                )
            )
        else:
            self.get_logger().info(
                'tester random initialpose published: x=%.3f y=%.3f yaw=%.3f'
                % (x, y, yaw)
            )

    def _begin_case_on_start(self) -> None:
        """Starts one test case when start command is sent."""
        if self._finished or self._shutdown_requested:
            return
        if len(self._results) >= self._max_tests:
            return
        if self._case_active:
            return
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        self._case_active = True
        self._case_start_t = t
        self._case_index += 1
        self._await_case_running = False
        self.get_logger().info('case-%d started at %.2fs (by start)' % (self._case_index, t))

    def _on_inter_case_start(self) -> None:
        if self._inter_case_start_timer is not None:
            self._inter_case_start_timer.cancel()
            self._inter_case_start_timer = None
        self._publish_start()
        self._await_case_running = True
        self.get_logger().info('tester next-case start published')

    def _schedule_next_case(self) -> None:
        # Do not start another case when enough rounds are already recorded.
        if self._start_after_case and (len(self._results) < self._max_tests):
            if self._inter_case_start_timer is not None:
                self._inter_case_start_timer.cancel()
            self._inter_case_start_timer = self.create_timer(
                self._inter_case_start_delay_s,
                self._on_inter_case_start,
            )

    def _cb_state(self, msg: String) -> None:
        self._latest_state_topic = str(msg.data).strip()
        state = str(self._latest_state_topic).upper()
        if self._finished or self._shutdown_requested:
            return
        now_terminal = (state == self._success_state) or (state in self._fail_states)
        # 与 docking_node 周期发布的非终态对齐：清掉“等非终态 metrics”的锁。
        # 否则若 /dock/test_metrics 只在终态才发一条，_await_case_running 会永远卡在终态 return。
        if not now_terminal:
            self._await_case_running = False
            return
        if len(self._results) >= self._max_tests:
            return
        # Do not finish case on state callback to avoid race:
        # /dock/state may arrive before terminal /dock/test_metrics sample,
        # which would make this round's metrics window empty.

    def _cb_metrics(self, msg: String) -> None:
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        items = [p.strip() for p in str(msg.data).split(',') if p.strip()]
        parsed: dict[str, str] = {'t_s': f'{t:.3f}'}
        for item in items:
            if '=' not in item:
                continue
            k, v = item.split('=', 1)
            parsed[k.strip()] = v.strip()
        if self._latest_state_topic:
            parsed['state_topic'] = self._latest_state_topic
        self._rows.append(parsed)
        state_dbg = self._extract_state(parsed)
        if state_dbg == self._success_state or state_dbg in self._fail_states:
            self.get_logger().info(
                'terminal metrics received: state=%s left=%s front=%s t=%.3f'
                % (
                    state_dbg,
                    parsed.get('lateral_front_y_to_dock_center_y_m', parsed.get('left_ir_to_dock_right_mid_m', 'N/A')),
                    parsed.get('front_contact_to_dock_center_m', 'N/A'),
                    t,
                )
            )
        self._update_case_state(parsed, t)
        self._dirty = True

    def _on_timer_save(self) -> None:
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9
        self._check_case_timeout(t)
        if not self._dirty or not self._rows:
            return
        self._save_csv()
        self._save_summary()
        if self._plot_enable:
            self._save_plot()
        self._dirty = False

    def _save_csv(self) -> None:
        keys = sorted({k for row in self._rows for k in row.keys()})
        with open(self._csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self._rows)

    def _save_plot(self) -> None:
        t = [float(row.get('t_s', '0.0')) for row in self._rows]
        d1 = [
            self._to_float(
                row.get('lateral_front_y_to_dock_center_y_m')
                or row.get('left_ir_to_dock_right_mid_m')
            )
            for row in self._rows
        ]
        d2 = [
            self._to_float(row.get('front_contact_to_dock_center_m')) for row in self._rows
        ]
        ok1 = [v is not None for v in d1]
        ok2 = [v is not None for v in d2]
        plt.figure(figsize=(10, 5))
        if any(ok1):
            plt.plot(
                [x for x, m in zip(t, ok1) if m],
                [y for y in d1 if y is not None],
                label='lateral_front_y_to_dock_center_y_m',
            )
        if any(ok2):
            plt.plot(
                [x for x, m in zip(t, ok2) if m],
                [y for y in d2 if y is not None],
                label='front_contact_to_dock_center_m',
            )
        plt.xlabel('time (s)')
        plt.ylabel('distance (m)')
        plt.title('Docking Metrics')
        plt.grid(True, alpha=0.3)
        plt.legend(loc='best')
        plt.tight_layout()
        plt.savefig(self._plot_path, dpi=140)
        plt.close()

    def _save_summary(self) -> None:
        keys = [
            'case_id',
            'start_t_s',
            'end_t_s',
            'duration_s',
            'result',
            'final_state',
            'grid_scenario',
            'longitudinal_m',
            'lateral_offset_m',
            'repeat_index',
            'start_x_m',
            'start_y_m',
            'start_yaw_rad',
        ]
        with open(self._summary_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self._results)

    def _save_group_success_summary(self) -> None:
        if not self._results:
            return
        groups: dict[tuple[str, str, str], list[str]] = {}
        for row in self._results:
            key = (
                str(row.get('grid_scenario', '')),
                str(row.get('longitudinal_m', '')),
                str(row.get('lateral_offset_m', '')),
            )
            groups.setdefault(key, []).append(str(row.get('result', '')))
        keys = [
            'grid_scenario',
            'longitudinal_m',
            'lateral_offset_m',
            'total',
            'success',
            'success_rate_pct',
        ]
        summary_rows: list[dict[str, str]] = []
        for (scenario, long_m, lat_m), results in sorted(groups.items()):
            total = len(results)
            success = sum(1 for r in results if r == 'success')
            rate = 100.0 * success / max(1, total)
            summary_rows.append(
                {
                    'grid_scenario': scenario,
                    'longitudinal_m': long_m,
                    'lateral_offset_m': lat_m,
                    'total': str(total),
                    'success': str(success),
                    'success_rate_pct': f'{rate:.1f}',
                }
            )
        with open(self._group_summary_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(summary_rows)
        if self._pose_schedule == 'grid':
            overall = sum(1 for r in self._results if r['result'] == 'success')
            self.get_logger().info(
                'success rate by lateral offset written to %s (overall %d/%d = %.1f%%)'
                % (
                    self._group_summary_path,
                    overall,
                    len(self._results),
                    100.0 * overall / max(1, len(self._results)),
                )
            )
            for row in summary_rows:
                self.get_logger().info(
                    '  offset %s m (long %s m): %s/%s = %s%%'
                    % (
                        row['lateral_offset_m'],
                        row['longitudinal_m'],
                        row['success'],
                        row['total'],
                        row['success_rate_pct'],
                    )
                )

    def _row_is_success_sample(self, row: dict[str, str]) -> bool:
        st = (
            str(row.get('state') or row.get('state_topic') or '')
            .strip()
            .upper()
        )
        return st == self._success_state

    def _compute_two_avg_errors_for_rows(
        self, rows: list[dict[str, str]]
    ) -> dict[str, str]:
        """距离两项误差 + yaw绝对偏差统计（max/min/avg）。"""
        key_l = 'lateral_front_y_to_dock_center_y_m'
        key_l_fallback = 'left_ir_to_dock_right_mid_m'
        key_f = 'front_contact_to_dock_center_m'
        left_vals: list[float] = []
        front_vals: list[float] = []
        yaw_abs_deg_vals: list[float] = []
        yaw_abs_minus_180_deg_vals: list[float] = []
        for row in rows:
            if self._eval_errors_success_only and (not self._row_is_success_sample(row)):
                continue
            lv = self._to_float(row.get(key_l) or row.get(key_l_fallback))
            fv = self._to_float(row.get(key_f))
            yaw_abs_deg = self._to_float(row.get('yaw_abs_err_to_dock_deg'))
            if yaw_abs_deg is None:
                yaw_abs_rad = self._to_float(row.get('yaw_abs_err_to_dock_rad'))
                if yaw_abs_rad is not None:
                    yaw_abs_deg = math.degrees(float(yaw_abs_rad))
            if lv is not None:
                left_vals.append(float(lv))
            if fv is not None and float(fv) >= 0.0:
                front_vals.append(float(fv))
            if yaw_abs_deg is not None and float(yaw_abs_deg) >= 0.0:
                yaw_abs_deg_f = float(yaw_abs_deg)
                yaw_abs_deg_vals.append(yaw_abs_deg_f)
                yaw_abs_minus_180_deg_vals.append(abs(yaw_abs_deg_f - 180.0))
        def _mean(xs: list[float]) -> Optional[float]:
            return sum(xs) / len(xs) if xs else None

        mean_l = _mean(left_vals)
        mean_f = _mean(front_vals)
        mean_yaw_abs_deg = _mean(yaw_abs_deg_vals)
        min_yaw_abs_deg = min(yaw_abs_deg_vals) if yaw_abs_deg_vals else None
        max_yaw_abs_deg = max(yaw_abs_deg_vals) if yaw_abs_deg_vals else None
        mean_yaw_abs_minus_180_deg = _mean(yaw_abs_minus_180_deg_vals)
        min_yaw_abs_minus_180_deg = (
            min(yaw_abs_minus_180_deg_vals) if yaw_abs_minus_180_deg_vals else None
        )
        max_yaw_abs_minus_180_deg = (
            max(yaw_abs_minus_180_deg_vals) if yaw_abs_minus_180_deg_vals else None
        )
        avg_err_l = (
            sum(abs(x - self._target_left_ir_m) for x in left_vals) / len(left_vals)
            if left_vals
            else None
        )
        avg_err_f = (
            sum(abs(x - self._target_front_m) for x in front_vals) / len(front_vals)
            if front_vals
            else None
        )
        return {
            'n_rows_metrics': str(len(rows)),
            'n_samples_left': str(len(left_vals)),
            'n_samples_front': str(len(front_vals)),
            'n_samples_yaw': str(len(yaw_abs_deg_vals)),
            'mean_left_ir_to_dock_right_mid_m': ''
            if mean_l is None
            else f'{mean_l:.6f}',
            'mean_front_contact_to_dock_center_m': ''
            if mean_f is None
            else f'{mean_f:.6f}',
            'avg_yaw_abs_err_deg': ''
            if mean_yaw_abs_deg is None
            else f'{mean_yaw_abs_deg:.6f}',
            'min_yaw_abs_err_deg': ''
            if min_yaw_abs_deg is None
            else f'{min_yaw_abs_deg:.6f}',
            'max_yaw_abs_err_deg': ''
            if max_yaw_abs_deg is None
            else f'{max_yaw_abs_deg:.6f}',
            'avg_yaw_abs_err_rad': ''
            if mean_yaw_abs_deg is None
            else f'{math.radians(mean_yaw_abs_deg):.6f}',
            'min_yaw_abs_err_rad': ''
            if min_yaw_abs_deg is None
            else f'{math.radians(min_yaw_abs_deg):.6f}',
            'max_yaw_abs_err_rad': ''
            if max_yaw_abs_deg is None
            else f'{math.radians(max_yaw_abs_deg):.6f}',
            'avg_yaw_abs_err_minus_180_deg': ''
            if mean_yaw_abs_minus_180_deg is None
            else f'{mean_yaw_abs_minus_180_deg:.6f}',
            'min_yaw_abs_err_minus_180_deg': ''
            if min_yaw_abs_minus_180_deg is None
            else f'{min_yaw_abs_minus_180_deg:.6f}',
            'max_yaw_abs_err_minus_180_deg': ''
            if max_yaw_abs_minus_180_deg is None
            else f'{max_yaw_abs_minus_180_deg:.6f}',
            'avg_yaw_abs_err_minus_180_rad': ''
            if mean_yaw_abs_minus_180_deg is None
            else f'{math.radians(mean_yaw_abs_minus_180_deg):.6f}',
            'min_yaw_abs_err_minus_180_rad': ''
            if min_yaw_abs_minus_180_deg is None
            else f'{math.radians(min_yaw_abs_minus_180_deg):.6f}',
            'max_yaw_abs_err_minus_180_rad': ''
            if max_yaw_abs_minus_180_deg is None
            else f'{math.radians(max_yaw_abs_minus_180_deg):.6f}',
            'avg_abs_err_left_ir_to_dock_right_mid_m': ''
            if avg_err_l is None
            else f'{avg_err_l:.6f}',
            'avg_abs_err_front_contact_to_dock_center_m': ''
            if avg_err_f is None
            else f'{avg_err_f:.6f}',
            'target_left_ir_to_dock_right_mid_m': f'{self._target_left_ir_m:.6f}',
            'target_front_contact_to_dock_center_m': f'{self._target_front_m:.6f}',
            'eval_errors_success_rows_only': str(self._eval_errors_success_only),
        }

    def _compute_two_avg_errors(self) -> dict[str, str]:
        return self._compute_two_avg_errors_for_rows(self._rows)

    def _save_evaluation(self) -> None:
        ev = self._compute_two_avg_errors()
        keys = list(ev.keys())
        with open(self._eval_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerow(ev)

    def _rows_for_case_time_window(self, case_row: dict[str, str]) -> list[dict[str, str]]:
        try:
            t0 = float(case_row['start_t_s'])
            t1 = float(case_row['end_t_s'])
        except (KeyError, ValueError):
            return []
        out: list[dict[str, str]] = []
        for row in self._rows:
            try:
                ts = float(row.get('t_s', 'nan'))
            except (TypeError, ValueError):
                continue
            if t0 <= ts <= t1:
                out.append(row)
        return out

    def _append_round_evaluation(self, case_row: dict[str, str]) -> None:
        """每轮结束：reset/start 前写入本窗内两项平均误差。"""
        sub = self._rows_for_case_time_window(case_row)
        ev = self._compute_two_avg_errors_for_rows(sub)
        # 兜底：若该轮判定成功，但由于状态字段缺失/不同步导致 success-only 过滤后无样本，
        # 自动回退为使用本轮全部样本，避免输出 N/A。
        case_result = str(case_row.get('result', '')).strip().lower()
        if (
            self._eval_errors_success_only
            and case_result == 'success'
            and ev.get('n_samples_left') == '0'
            and ev.get('n_samples_front') == '0'
        ):
            left_vals: list[float] = []
            front_vals: list[float] = []
            yaw_abs_deg_vals: list[float] = []
            yaw_abs_minus_180_deg_vals: list[float] = []
            for row_item in sub:
                lv = self._to_float(
                    row_item.get('lateral_front_y_to_dock_center_y_m')
                    or row_item.get('left_ir_to_dock_right_mid_m')
                )
                fv = self._to_float(row_item.get('front_contact_to_dock_center_m'))
                yaw_abs_deg = self._to_float(row_item.get('yaw_abs_err_to_dock_deg'))
                if yaw_abs_deg is None:
                    yaw_abs_rad = self._to_float(row_item.get('yaw_abs_err_to_dock_rad'))
                    if yaw_abs_rad is not None:
                        yaw_abs_deg = math.degrees(float(yaw_abs_rad))
                if lv is not None:
                    left_vals.append(float(lv))
                if fv is not None and float(fv) >= 0.0:
                    front_vals.append(float(fv))
                if yaw_abs_deg is not None and float(yaw_abs_deg) >= 0.0:
                    yaw_abs_deg_f = float(yaw_abs_deg)
                    yaw_abs_deg_vals.append(yaw_abs_deg_f)
                    yaw_abs_minus_180_deg_vals.append(abs(yaw_abs_deg_f - 180.0))
            if left_vals or front_vals or yaw_abs_deg_vals:
                def _mean(xs: list[float]) -> Optional[float]:
                    return sum(xs) / len(xs) if xs else None

                mean_l = _mean(left_vals)
                mean_f = _mean(front_vals)
                mean_yaw_abs_deg = _mean(yaw_abs_deg_vals)
                min_yaw_abs_deg = min(yaw_abs_deg_vals) if yaw_abs_deg_vals else None
                max_yaw_abs_deg = max(yaw_abs_deg_vals) if yaw_abs_deg_vals else None
                mean_yaw_abs_minus_180_deg = _mean(yaw_abs_minus_180_deg_vals)
                min_yaw_abs_minus_180_deg = (
                    min(yaw_abs_minus_180_deg_vals) if yaw_abs_minus_180_deg_vals else None
                )
                max_yaw_abs_minus_180_deg = (
                    max(yaw_abs_minus_180_deg_vals) if yaw_abs_minus_180_deg_vals else None
                )
                avg_err_l = (
                    sum(abs(x - self._target_left_ir_m) for x in left_vals) / len(left_vals)
                    if left_vals
                    else None
                )
                avg_err_f = (
                    sum(abs(x - self._target_front_m) for x in front_vals) / len(front_vals)
                    if front_vals
                    else None
                )
                ev.update(
                    {
                        'n_samples_left': str(len(left_vals)),
                        'n_samples_front': str(len(front_vals)),
                        'n_samples_yaw': str(len(yaw_abs_deg_vals)),
                        'mean_left_ir_to_dock_right_mid_m': ''
                        if mean_l is None
                        else f'{mean_l:.6f}',
                        'mean_front_contact_to_dock_center_m': ''
                        if mean_f is None
                        else f'{mean_f:.6f}',
                        'avg_yaw_abs_err_deg': ''
                        if mean_yaw_abs_deg is None
                        else f'{mean_yaw_abs_deg:.6f}',
                        'min_yaw_abs_err_deg': ''
                        if min_yaw_abs_deg is None
                        else f'{min_yaw_abs_deg:.6f}',
                        'max_yaw_abs_err_deg': ''
                        if max_yaw_abs_deg is None
                        else f'{max_yaw_abs_deg:.6f}',
                        'avg_yaw_abs_err_rad': ''
                        if mean_yaw_abs_deg is None
                        else f'{math.radians(mean_yaw_abs_deg):.6f}',
                        'min_yaw_abs_err_rad': ''
                        if min_yaw_abs_deg is None
                        else f'{math.radians(min_yaw_abs_deg):.6f}',
                        'max_yaw_abs_err_rad': ''
                        if max_yaw_abs_deg is None
                        else f'{math.radians(max_yaw_abs_deg):.6f}',
                        'avg_yaw_abs_err_minus_180_deg': ''
                        if mean_yaw_abs_minus_180_deg is None
                        else f'{mean_yaw_abs_minus_180_deg:.6f}',
                        'min_yaw_abs_err_minus_180_deg': ''
                        if min_yaw_abs_minus_180_deg is None
                        else f'{min_yaw_abs_minus_180_deg:.6f}',
                        'max_yaw_abs_err_minus_180_deg': ''
                        if max_yaw_abs_minus_180_deg is None
                        else f'{max_yaw_abs_minus_180_deg:.6f}',
                        'avg_yaw_abs_err_minus_180_rad': ''
                        if mean_yaw_abs_minus_180_deg is None
                        else f'{math.radians(mean_yaw_abs_minus_180_deg):.6f}',
                        'min_yaw_abs_err_minus_180_rad': ''
                        if min_yaw_abs_minus_180_deg is None
                        else f'{math.radians(min_yaw_abs_minus_180_deg):.6f}',
                        'max_yaw_abs_err_minus_180_rad': ''
                        if max_yaw_abs_minus_180_deg is None
                        else f'{math.radians(max_yaw_abs_minus_180_deg):.6f}',
                        'avg_abs_err_left_ir_to_dock_right_mid_m': ''
                        if avg_err_l is None
                        else f'{avg_err_l:.6f}',
                        'avg_abs_err_front_contact_to_dock_center_m': ''
                        if avg_err_f is None
                        else f'{avg_err_f:.6f}',
                        'eval_errors_success_rows_only': 'False(auto_fallback)',
                    }
                )
        row = {
            'case_id': case_row.get('case_id', ''),
            'result': case_row.get('result', ''),
            'final_state': case_row.get('final_state', ''),
            **ev,
        }
        keys = list(row.keys())
        mode = 'a' if self._eval_rounds_header_written else 'w'
        with open(self._eval_rounds_path, mode, newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            if not self._eval_rounds_header_written:
                writer.writeheader()
                self._eval_rounds_header_written = True
            writer.writerow(row)
        self.get_logger().info(
            'round-%s eval: avg_abs_err_left=%s avg_abs_err_front=%s yaw_abs_minus180(avg/min/max)_deg=%s/%s/%s (n_left=%s n_front=%s n_yaw=%s)'
            % (
                row['case_id'],
                ev['avg_abs_err_left_ir_to_dock_right_mid_m'] or 'N/A',
                ev['avg_abs_err_front_contact_to_dock_center_m'] or 'N/A',
                ev.get('avg_yaw_abs_err_minus_180_deg') or 'N/A',
                ev.get('min_yaw_abs_err_minus_180_deg') or 'N/A',
                ev.get('max_yaw_abs_err_minus_180_deg') or 'N/A',
                ev['n_samples_left'],
                ev['n_samples_front'],
                ev.get('n_samples_yaw', '0'),
            )
        )

    def _update_case_state(self, parsed: dict[str, str], t: float) -> None:
        if self._finished or self._shutdown_requested:
            return
        if len(self._results) >= self._max_tests:
            return
        state = self._extract_state(parsed)
        is_terminal = (state == self._success_state) or (state in self._fail_states)
        # Ignore stale terminal state after reset/start. Wait until we see
        # non-terminal running state before counting next case.
        if self._await_case_running:
            if is_terminal:
                return
            self._await_case_running = False
        if not self._case_active or self._case_start_t is None:
            return
        if state == self._success_state:
            self._finish_case('success', state, t)
            return
        if state in self._fail_states:
            self._finish_case('fail_state', state, t)

    def _check_case_timeout(self, t: float) -> None:
        if (not self._case_active) or self._case_start_t is None:
            return
        if (t - self._case_start_t) >= self._case_timeout_s:
            self._finish_case('timeout', '', t)

    def _finish_case(
        self,
        result: str,
        final_state: str,
        end_t: float,
        *,
        instant_same_tick: bool = False,
    ) -> None:
        if len(self._results) >= self._max_tests:
            return
        if self._case_start_t is None:
            return
        duration = max(0.0, end_t - self._case_start_t)
        # Protect against stale terminal-state carry-over being counted
        # as an instant successful/failed case right after reset/start.
        if (
            (not instant_same_tick)
            and duration < self._min_case_duration_s
        ):
            self._case_active = False
            self._case_start_t = None
            self._await_case_running = True
            self._case_index = max(0, self._case_index - 1)
            self.get_logger().info(
                'ignore short case: duration=%.2fs < min_case_duration_s=%.2fs'
                % (duration, self._min_case_duration_s)
            )
            self._schedule_next_case()
            return
        start_t = self._case_start_t
        row = {
            'case_id': str(self._case_index),
            'start_t_s': f'{start_t:.3f}',
            'end_t_s': f'{end_t:.3f}',
            'duration_s': f'{duration:.3f}',
            'result': result,
            'final_state': final_state,
            'grid_scenario': self._case_pose_meta.get('grid_scenario', ''),
            'longitudinal_m': self._case_pose_meta.get('longitudinal_m', ''),
            'lateral_offset_m': self._case_pose_meta.get('lateral_offset_m', ''),
            'repeat_index': self._case_pose_meta.get('repeat_index', ''),
            'start_x_m': self._case_pose_meta.get('start_x_m', ''),
            'start_y_m': self._case_pose_meta.get('start_y_m', ''),
            'start_yaw_rad': self._case_pose_meta.get('start_yaw_rad', ''),
        }
        self._results.append(row)
        self._case_pose_meta = {}
        self._case_active = False
        self._case_start_t = None
        self._dirty = True
        self.get_logger().info(
            'case-%d finished: result=%s duration=%.2fs state=%s'
            % (self._case_index, result, duration, final_state or 'N/A')
        )
        # 本轮统计 -> 再 reset -> start 进入下一轮（由 _schedule_next_case 触发）
        self._append_round_evaluation(row)
        if self._reset_after_case:
            self._publish_reset()
        self._schedule_next_case()
        self._maybe_all_done()

    def _maybe_all_done(self) -> None:
        n_done = len(self._results)
        if n_done < self._max_tests:
            return
        self._finished = True
        succ = sum(1 for r in self._results if r['result'] == 'success')
        self.get_logger().info(
            'all tests done: recorded=%d success=%d success_rate=%.1f%%'
            % (n_done, succ, 100.0 * succ / max(1, n_done))
        )
        ev = self._compute_two_avg_errors()
        self.get_logger().info(
            'eval avg_abs_err: left_ir_to_dock_right_mid_m=%s m, '
            'front_contact_to_dock_center_m=%s m, '
            'yaw_abs_err_minus180_deg(avg/min/max)=%s/%s/%s '
            '(targets=%.4f, %.4f, success_rows_only=%s, n_left=%s n_front=%s n_yaw=%s)'
            % (
                ev['avg_abs_err_left_ir_to_dock_right_mid_m'] or 'N/A',
                ev['avg_abs_err_front_contact_to_dock_center_m'] or 'N/A',
                ev.get('avg_yaw_abs_err_minus_180_deg') or 'N/A',
                ev.get('min_yaw_abs_err_minus_180_deg') or 'N/A',
                ev.get('max_yaw_abs_err_minus_180_deg') or 'N/A',
                self._target_left_ir_m,
                self._target_front_m,
                ev['eval_errors_success_rows_only'],
                ev['n_samples_left'],
                ev['n_samples_front'],
                ev.get('n_samples_yaw', '0'),
            )
        )
        self._save_csv()
        self._save_summary()
        self._save_group_success_summary()
        self._save_evaluation()
        if self._plot_enable:
            self._save_plot()
        if self._auto_stop_when_done:
            self._request_shutdown()

    def _request_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.get_logger().info('docking_tester requesting shutdown')
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
        self._shutdown_timer = self.create_timer(0.05, self._on_shutdown_timer)

    def _on_shutdown_timer(self) -> None:
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None
        # Stop spin from a timer callback; destroy_node happens in main() finally.
        rclpy.shutdown()

    def _extract_state(self, parsed: dict[str, str]) -> str:
        state = parsed.get('state') or parsed.get('state_topic') or ''
        return str(state).strip().upper()

    @staticmethod
    def _to_float(v: Optional[str]) -> Optional[float]:
        if v is None:
            return None
        try:
            return float(v)
        except (TypeError, ValueError):
            return None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DockingTester()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
