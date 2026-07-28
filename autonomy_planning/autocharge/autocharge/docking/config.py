"""Docking ROS parameter defaults and loaders."""

from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node

from autocharge.docking.ir import IrFilter
from autocharge.docking.utils import IrToken
from autocharge.docking.predock import PathFinderController, PredockNavigator
from autocharge.docking.type import DockGeometry, DockingConfig, PredockLimits, PredockNoise

# Keep ROS params minimal. Derived motion/timing knobs live on DockingConfig
# properties and PredockLimits defaults.
PARAM_DEFAULTS: dict[str, object] = {
    'ir_dock_topic': '/ir/dock',
    'cmd_vel_topic': '/cmd_vel',
    'charge_topic': '/dock/charge_connected',
    'abort_topic': '/dock/abort',
    'start_topic': '/dock/start',
    'reset_topic': '/dock/reset',
    'start_on_boot': False,
    'start_reset_fsm': True,
    'reset_pose_on_reset': True,
    'initialpose_topic': '/initialpose',
    'reset_pose_x': 0.0,
    'reset_pose_y': 0.0,
    'reset_pose_yaw': 0.0,
    'odom_topic': '/odom',
    'dock_x': 1.0,
    'dock_y': 0.0,
    'dock_yaw': 0.0,
    'dock_front_x_offset_m': 0.0,
    'dock_front_y_offset_m': 0.0,
    'success_contact_x_m': 0.16,
    'success_contact_y_m': 0.0,
    'success_use_distance': True,
    'fail_on_dock_overrun_enable': True,
    'fail_on_dock_overrun_margin_m': 0.0,
    'stats_enable': True,
    'stats_topic': '/dock/test_metrics',
    'state_topic': '/dock/state',
    'dock_to_charger_action': 'dock_to_charger',
    'action_default_timeout_s': 300.0,
    'ir_heading_gate_enable': False,
    'ir_back_reject_deg': 85.0,
    # predock (optional; strategy 1 keeps enable_nav=false)
    'predock.enable_nav': False,
    'predock.distance_m': 0.5,
    'predock.position_noise_std_m': 0.005,
    'predock.yaw_noise_deg': 10.0,
    'predock.goal_pos_tol_m': 0.05,
    'predock.goal_yaw_tol_deg': 5.0,
    'predock.max_linear_speed': 0.08,
    'predock.max_angular_speed': 0.20,
    'predock.publish_pose': True,
    'predock.pose_topic': '/predock_pose',
    'control_hz': 20.0,
    'ir_eval_every_cycles': 0,
    'control_log': True,
    'cmd_vel_alpha': 0.7,
    # IR token map
    'docking.bit_left': IrToken.BIT_L1,
    'docking.bit_left_ext': IrToken.BIT_L2,
    'docking.bit_center': IrToken.BIT_C,
    'docking.bit_right': IrToken.BIT_R1,
    'docking.bit_right_ext': IrToken.BIT_R2,
    # FSM motion / IR timing (minimal)
    'docking.v_forward': 0.05,
    'docking.center_trim_wz': 0.05,
    'docking.search_wz': 0.25,
    'docking.bias_track_vx': 0.022,
    'docking.bias_track_wz': 0.22,
    'docking.ir_decision_period_s': 0.20,
    'docking.state_confirm_samples': 2,
    'docking.no_signal_search_timeout_s': 0.75,
}


def declare_parameters(node: Node) -> None:
    for name, value in PARAM_DEFAULTS.items():
        node.declare_parameter(name, value)


def load_fsm_config(node: Node) -> DockingConfig:
    p = node.get_parameter
    return DockingConfig(
        token_mask=0x1F,
        bit_l1=int(p('docking.bit_left').value),
        bit_l2=int(p('docking.bit_left_ext').value),
        bit_c=int(p('docking.bit_center').value),
        bit_r2=int(p('docking.bit_right').value),
        bit_r1=int(p('docking.bit_right_ext').value),
        center_vx=float(p('docking.v_forward').value),
        center_trim_wz=abs(float(p('docking.center_trim_wz').value)),
        search_wz=abs(float(p('docking.search_wz').value)),
        bias_track_vx=abs(float(p('docking.bias_track_vx').value)),
        bias_track_wz=abs(float(p('docking.bias_track_wz').value)),
        ir_decision_period_s=float(p('docking.ir_decision_period_s').value),
        state_confirm_samples=int(p('docking.state_confirm_samples').value),
        no_signal_search_timeout_s=float(p('docking.no_signal_search_timeout_s').value),
    )


def load_geometry(node: Node) -> DockGeometry:
    p = node.get_parameter
    geom = DockGeometry(
        dock_x=float(p('dock_x').value),
        dock_y=float(p('dock_y').value),
        dock_yaw=float(p('dock_yaw').value),
        dock_front_x_offset_m=float(p('dock_front_x_offset_m').value),
        dock_front_y_offset_m=float(p('dock_front_y_offset_m').value),
        success_contact_x_m=float(p('success_contact_x_m').value),
        success_contact_y_m=float(p('success_contact_y_m').value),
        success_use_distance=bool(p('success_use_distance').value),
    )
    geom.capture_defaults()
    return geom


def load_predock_noise(node: Node) -> PredockNoise:
    p = node.get_parameter
    return PredockNoise(
        position_std_m=max(0.0, float(p('predock.position_noise_std_m').value)),
        yaw_deg=max(0.0, float(p('predock.yaw_noise_deg').value)),
    )


def load_predock_navigator(node: Node) -> PredockNavigator:
    p = node.get_parameter
    max_lin = max(0.01, float(p('predock.max_linear_speed').value))
    max_ang = max(0.05, float(p('predock.max_angular_speed').value))
    limits = PredockLimits(
        goal_pos_tol_m=max(0.005, float(p('predock.goal_pos_tol_m').value)),
        goal_yaw_tol_deg=max(1.0, float(p('predock.goal_yaw_tol_deg').value)),
        max_linear_speed=max_lin,
        max_angular_speed=max_ang,
        # Soft gains derived from speed caps (no separate ROS params).
        yaw_finish_kp=max(0.5, 5.0 * max_ang),
        min_linear_speed=min(0.02, 0.25 * max_lin),
        near_goal_min_linear_speed=min(0.02, 0.25 * max_lin),
    )
    # Path-finder gains scaled from speed caps.
    controller = PathFinderController(
        kp_rho=max(0.3, 6.5 * max_lin),
        kp_alpha=max(0.6, 5.0 * max_ang),
        kp_beta=max(0.3, 2.25 * max_ang),
    )
    return PredockNavigator(limits, controller)


def load_ir_filter(node: Node, fsm_cfg: DockingConfig) -> IrFilter:
    p = node.get_parameter
    return IrFilter(
        fsm_cfg,
        heading_gate_enable=bool(p('ir_heading_gate_enable').value),
        back_reject_deg=float(p('ir_back_reject_deg').value),
    )


@dataclass(frozen=True)
class DockNodeRuntime:
    geom: DockGeometry
    fsm_cfg: DockingConfig
    ir_filter: IrFilter
    predock_navigator: PredockNavigator
    predock_noise: PredockNoise
    predock_enable_nav: bool
    predock_distance_m: float
    stats_enable: bool
    control_log: bool
    start_reset_fsm: bool
    reset_pose_x: float
    reset_pose_y: float
    reset_pose_yaw: float
    reset_pose_on_reset: bool
    start_on_boot: bool
    control_hz: float
    ir_eval_every_cycles: int
    ir_topic: str
    fail_on_dock_overrun_enable: bool
    fail_on_dock_overrun_margin_m: float


def load_runtime(node: Node) -> DockNodeRuntime:
    fsm_cfg = load_fsm_config(node)
    control_hz = max(1.0, float(node.get_parameter('control_hz').value))
    eval_every = int(node.get_parameter('ir_eval_every_cycles').value)
    if eval_every <= 0:
        eval_every = max(1, int(round(control_hz / 4.0)))
    p = node.get_parameter
    return DockNodeRuntime(
        geom=load_geometry(node),
        fsm_cfg=fsm_cfg,
        ir_filter=load_ir_filter(node, fsm_cfg),
        predock_navigator=load_predock_navigator(node),
        predock_noise=load_predock_noise(node),
        predock_enable_nav=bool(p('predock.enable_nav').value),
        predock_distance_m=max(0.0, float(p('predock.distance_m').value)),
        stats_enable=bool(p('stats_enable').value),
        control_log=bool(p('control_log').value),
        start_reset_fsm=bool(p('start_reset_fsm').value),
        reset_pose_x=float(p('reset_pose_x').value),
        reset_pose_y=float(p('reset_pose_y').value),
        reset_pose_yaw=float(p('reset_pose_yaw').value),
        reset_pose_on_reset=bool(p('reset_pose_on_reset').value),
        start_on_boot=bool(p('start_on_boot').value),
        control_hz=control_hz,
        ir_eval_every_cycles=int(eval_every),
        ir_topic=str(p('ir_dock_topic').value).strip() or '/ir/dock',
        fail_on_dock_overrun_enable=bool(p('fail_on_dock_overrun_enable').value),
        fail_on_dock_overrun_margin_m=max(
            0.0, float(p('fail_on_dock_overrun_margin_m').value)
        ),
    )
