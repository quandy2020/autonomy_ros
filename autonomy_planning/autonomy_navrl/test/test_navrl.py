"""Unit tests for autonomy_navrl modules."""

from __future__ import annotations

import numpy as np
import pytest

from autonomy_navrl.control.commands import scale_action_to_command
from autonomy_navrl.env.mock_env import MockNavrlEnv
from autonomy_navrl.preprocessing.odom import goal_relative_state
from autonomy_navrl.preprocessing.rgbd import depth_collision_score


def test_scale_action_to_command():
    command = scale_action_to_command(np.array([1.0, 0.0, -1.0]), 1.0, 0.5, 1.5)
    assert command.vx > 0.0
    assert command.w < 0.0


def test_goal_relative_state():
    state = goal_relative_state(
        np.array([0.0, 0.0], dtype=np.float32),
        0.0,
        np.array([1.0, 0.0], dtype=np.float32),
    )
    assert state[2] == 1.0


def test_depth_collision_score():
    depth = np.full((96, 128), 2.0, dtype=np.float32)
    depth[40:60, 50:70] = 0.2
    score = depth_collision_score(depth, threshold_m=0.35)
    assert score > 0.0


def test_mock_env_step():
    config = {
        'num_envs': 2,
        'seed': 0,
        'sensors': {
            'rgb': {'width': 32, 'height': 24, 'fov_deg': 90.0},
            'depth': {'max_range_m': 5.0, 'collision_threshold_m': 0.35},
        },
        'control': {'max_vx': 1.0, 'max_vy': 0.5, 'max_w': 1.5},
        'task': {
            'goal_tolerance_m': 0.5,
            'max_episode_steps': 10,
            'arena_size_m': 8.0,
        },
        'reward': {},
        'policy': {'state_dim': 19},
    }
    env = MockNavrlEnv(config)
    obs = env.reset()
    assert obs['rgbd'].shape[0] == 2
    assert np.allclose(obs['rgbd'], 0.0)
    result = env.step(np.zeros((2, 3), dtype=np.float32))
    assert result.reward.shape == (2,)
    env.close()


def test_precision_controller_full_alignment():
    from autonomy_navrl.control.precision_controller import (
        compute_pose_tracking_action,
        is_fully_aligned,
    )

    action = compute_pose_tracking_action(
        0.1, 0.02, 0.12, 0.05, 0.04,
        max_vx=0.5, max_vy=0.5, max_w=1.0,
    )
    assert action.shape == (3,)
    assert np.all(action >= -1.0) and np.all(action <= 1.0)
    assert is_fully_aligned(0.04, 0.05, 0.05, pos_tol_m=0.05, yaw_tol_rad=0.1, speed_tol_mps=0.08)
    assert not is_fully_aligned(0.10, 0.05, 0.05, pos_tol_m=0.05, yaw_tol_rad=0.1, speed_tol_mps=0.08)


def test_precision_controller_yaw_priority():
    from autonomy_navrl.control.precision_controller import compute_pose_tracking_action

    # Inside yaw-lock distance with large yaw error → translation frozen.
    action = compute_pose_tracking_action(
        0.05, 0.02, 0.10, 0.1, 0.28,
        max_vx=0.5, max_vy=0.5, max_w=1.0,
    )
    assert action.shape == (3,)
    assert abs(action[0]) < 0.05 and abs(action[1]) < 0.05
    assert action[2] != 0.0


def test_precision_controller_face_then_go():
    from autonomy_navrl.control.precision_controller import compute_pose_tracking_action

    # Far + goal to the side → arc-turn (vx + w), do not strafe.
    action = compute_pose_tracking_action(
        0.14, 4.0, 4.0, 1.54, 0.2,
        max_vx=0.5, max_vy=0.3, max_w=1.0,
    )
    assert abs(action[1]) < 0.05  # no long-range crab
    assert abs(action[2]) >= 0.5  # strong turn toward bearing
    assert abs(action[0]) > 0.05  # forward roll assist for scrubbing turn


def test_precision_parking_reward_computer():
    torch = pytest.importorskip('torch')
    from autonomy_navrl.plugins.bootstrap import ensure_plugins
    from autonomy_navrl.plugins.go2w.rewards import PrecisionParkingConfig, PrecisionParkingRewardComputer
    from autonomy_navrl.plugins.rewards import RewardStepContext

    ensure_plugins()

    class _Cfg:
        goal_tolerance_m = 0.08
        yaw_tolerance_rad = 0.12
        reward_progress_scale = 1.0
        reward_collision_penalty = -5.0
        reward_timeout_penalty = -1.0
        _reward_raw = {'precision_parking': {}}
        _framework = None

    class _Task:
        def _body_goal_features(self, pos_xy, yaw):
            del pos_xy, yaw
            return (
                torch.tensor([0.05, 0.02, 0.18, 0.10]),
                torch.tensor([0.01, 0.01, 0.05, 0.02]),
                torch.tensor([0.18, 0.50, 1.0, 2.0]),
                torch.tensor([0.1, 0.2, 0.5, 1.0]),
                torch.tensor([0.28, 0.08, 0.20, 0.50]),
            )

        def modify_reward(self, reward, *, distance, yaw_error):
            del distance, yaw_error
            return reward

    class _Env:
        device = torch.device('cpu')
        cfg = _Cfg()
        _task = _Task()
        _goals = torch.zeros(4, 2)

        def _robot_pos_xy(self):
            return torch.zeros(4, 2)

        def _robot_yaw(self):
            return torch.zeros(4)

    env = _Env()
    computer = PrecisionParkingRewardComputer(env)
    n = 4
    ctx = RewardStepContext(
        distance=torch.tensor([0.18, 0.50, 1.0, 2.0]),
        progress=torch.ones(n) * 0.01,
        proximity=torch.zeros(n),
        bearing=torch.zeros(n),
        yaw_error=torch.tensor([0.28, 0.08, 0.20, 0.50]),
        reached=torch.tensor([False, False, False, False]),
        commands=torch.tensor([
            [0.0, 0.0, 0.8],
            [0.1, 0.0, 0.2],
            [0.3, 0.0, 0.0],
            [0.5, 0.0, 0.0],
        ]),
        prev_commands=torch.zeros(n, 3),
        lin_speed_b=torch.zeros(n),
        collision_terminated=torch.zeros(n, dtype=torch.bool),
        timeout=torch.zeros(n, dtype=torch.bool),
    )
    reward, metrics = computer.compute(ctx)
    assert reward.shape == (n,)
    assert reward[0] > reward[2]
    assert metrics['yaw_error_mean'] > 0.0
    cfg = PrecisionParkingConfig.from_dict({})
    assert len(cfg.yaw_tanh) >= 3


def test_rgb_frame_to_bgr():
    from autonomy_navrl.viz.isaac.viewport_capture import rgb_frame_to_bgr

    frame = np.zeros((8, 12, 3), dtype=np.uint8)
    frame[:, :, 0] = 255
    bgr = rgb_frame_to_bgr(frame, scale=2)
    assert bgr.shape == (16, 24, 3)
    assert bgr[0, 0, 2] == 255


def test_jdrobot_reward_computer():
    torch = pytest.importorskip('torch')
    from autonomy_navrl.plugins.jdrobot.rewards import JdrobotRewardComputer, JdrobotRewardConfig

    device = torch.device('cpu')
    computer = JdrobotRewardComputer(JdrobotRewardConfig.from_dict({'profile': 'jdrobot'}), device)
    n = 4
    reward, metrics = computer.compute(
        distance=torch.tensor([0.05, 0.5, 1.0, 2.0]),
        bearing=torch.zeros(n),
        progress=torch.ones(n) * 0.1,
        proximity=torch.zeros(n),
        commands=torch.zeros(n, 3),
        prev_commands=torch.zeros(n, 3),
        lin_speed_b=torch.zeros(n),
        reached=torch.tensor([True, False, False, False]),
        collision_terminated=torch.zeros(n, dtype=torch.bool),
        timeout=torch.zeros(n, dtype=torch.bool),
    )
    assert reward.shape == (n,)
    assert reward[0] > reward[2]
    assert metrics['goal_rate'] == 0.25


def test_pose_viz_config():
    from autonomy_navrl.viz.isaac.config import PoseVizConfig

    cfg = PoseVizConfig.from_dict({'enabled': True, 'modules': ['target_footprint']})
    assert cfg.enabled
    assert cfg.modules == ['target_footprint']

    webrtc = PoseVizConfig.from_dict({'auto_enable_on_livestream': True}, livestream=2)
    assert webrtc.enabled

    with pytest.raises(ValueError, match='Unknown viz.modules'):
        PoseVizConfig.from_dict({'modules': ['not_a_module']})


def test_framework_config_robot_kinds():
    from autonomy_navrl.core.spec import FrameworkConfig

    cfg = FrameworkConfig.from_yaml_dict({
        'robot': {
            'kind': 'diff_drive',
            'urdf_path': '/tmp/robot.urdf',
        },
        'control': {},
        'sensors': {},
        'task': {'kind': 'goal_nav'},
    })
    assert cfg.robot.kind == 'diff_drive'
    assert cfg.action.model == 'diff_drive_2d'
    assert cfg.action.dim == 2


def test_robot_preset_merge():
    from autonomy_navrl.plugins.robots import apply_robot_preset

    raw = {'urdf_path': '/tmp/go2w.urdf'}
    apply_robot_preset('go2w', raw)
    assert raw['kind'] == 'wheeled_legged'
    assert raw['base_link'] == 'base'


def test_action_registry():
    from autonomy_navrl.plugins.actions import create_action_adapter
    from autonomy_navrl.core.spec import ActionSpec
    import torch

    spec = ActionSpec(model='holonomic_3d', dim=3)
    adapter = create_action_adapter(spec, num_envs=2, device='cpu')
    actions = torch.zeros(2, 3)
    adapter.pre_physics_step(actions)
    assert adapter.dim == 3


def test_reward_registry():
    from autonomy_navrl.plugins.rewards import reward_registry

    names = reward_registry.names()
    assert 'default' in names
    assert 'jdrobot' in names


def test_list_plugins():
    from autonomy_navrl.core.catalog import list_plugins

    catalog = list_plugins()
    assert 'goal_nav' in catalog['task_kinds']
    assert 'go2w_precision_pose' in catalog['task_kinds']
    assert 'waypoint_nav' in catalog['task_kinds']
    assert 'holonomic_3d' in catalog['action_models']


def test_locomotion_registry():
    from autonomy_navrl.plugins.bootstrap import ensure_plugins
    from autonomy_navrl.plugins.locomotion.base import locomotion_registry

    ensure_plugins()
    names = locomotion_registry.names()
    assert 'root_velocity' in names
    assert 'go2w_jit' in names
    assert 's10_jit' in names


def test_s10_joint_names():
    from autonomy_navrl.plugins.locomotion.s10_jit import S10_ALL_JOINT_NAMES

    assert len(S10_ALL_JOINT_NAMES) == 16
    assert S10_ALL_JOINT_NAMES[-1] == 'hr_wheel_joint'


def test_extra_reward_profiles():
    from autonomy_navrl.plugins.bootstrap import ensure_plugins

    ensure_plugins()
    from autonomy_navrl.plugins.rewards import reward_registry

    names = reward_registry.names()
    assert 'sparse' in names
    assert 'precision_iou' in names
    assert 'precision_parking' in names


def test_navrl_framework_dry_config():
    from autonomy_navrl.core.api import NavrlFramework

    fw = NavrlFramework({
        'robot': {'kind': 'quadruped', 'urdf_path': '/tmp/test.urdf'},
        'control': {},
        'sensors': {},
        'task': {'kind': 'goal_nav'},
        'num_envs': 4,
    })
    assert 'goal_nav' in fw.summary()
    assert fw.framework.robot.kind == 'quadruped'


def test_go2w_bbox_iou_perfect_alignment():
    torch = pytest.importorskip('torch')
    from autonomy_navrl.plugins.go2w.iou import bbox_iou_from_body_pose

    dx = torch.zeros(1)
    dy = torch.zeros(1)
    yaw = torch.zeros(1)
    iou = bbox_iou_from_body_pose(dx, dy, yaw)
    assert float(iou[0]) >= 0.99


def test_go2w_consecutive_success_tracker():
    torch = pytest.importorskip('torch')
    from autonomy_navrl.plugins.go2w.success_tracker import ConsecutiveIoUConfig, ConsecutiveIoUTracker

    cfg = ConsecutiveIoUConfig(enabled=True, iou_threshold=0.5, num_success=3)
    tracker = ConsecutiveIoUTracker(cfg, num_envs=1, device=torch.device('cpu'))
    dx = torch.zeros(1)
    dy = torch.zeros(1)
    yaw = torch.zeros(1)
    for _ in range(2):
        assert not tracker.update(dx, dy, yaw).item()
    assert tracker.update(dx, dy, yaw).item()


def test_go2w_config_base_merge():
    from pathlib import Path

    from autonomy_navrl.core.config_loader import load_yaml_config

    pkg = Path(__file__).resolve().parents[1]
    cfg = load_yaml_config(pkg / 'config' / 'go2w.yaml', profile='jit')
    assert cfg['robot']['preset'] == 'go2w'
    assert cfg['locomotion']['backend'] == 'go2w_jit'
    assert cfg['task']['kind'] == 'go2w_precision_pose'
    assert cfg['task']['consecutive_iou']['num_success'] == 10


def test_s10_config_profile_smoke():
    from pathlib import Path

    from autonomy_navrl.core.config_loader import load_yaml_config

    pkg = Path(__file__).resolve().parents[1]
    cfg = load_yaml_config(pkg / 'config' / 's10.yaml', profile='smoke')
    assert cfg['run_name'] == 'navrl_smoke'
    assert cfg['ppo']['total_timesteps'] == 256


def test_model_spec_from_config():
    from autonomy_navrl.models.spec import ModelSpec

    spec = ModelSpec.from_config({
        'policy': {'state_dim': 20, 'hidden_dim': 128},
        'sensors': {'rgb': {'width': 64, 'height': 48}},
    })
    assert spec.fusion == 'concat'
    assert spec.policy_type == 'gaussian_actor_critic'
    assert spec.image_width == 64


def test_algorithm_factory():
    from autonomy_navrl.algorithms.ppo import algorithm_registry

    import autonomy_navrl.algorithms.grpo  # noqa: F401
    import autonomy_navrl.algorithms.diffusion  # noqa: F401

    names = algorithm_registry.names()
    assert 'ppo' in names
    assert 'grpo' in names
    assert 'diffusion' in names

    from autonomy_navrl.algorithms.factory import resolve_algorithm_name
    assert resolve_algorithm_name({'algorithm': {'name': 'ppo'}}) == 'ppo'


def test_create_nav_policy_smoke():
    torch = pytest.importorskip('torch')
    from autonomy_navrl.models.factory import create_nav_policy

    policy = create_nav_policy({
        'policy': {'state_dim': 20, 'hidden_dim': 64},
        'sensors': {'rgb': {'width': 32, 'height': 24}},
    })
    rgbd = torch.zeros(2, 4, 24, 32)
    state = torch.zeros(2, 20)
    action, log_prob, value, _ = policy.act_tensors(rgbd, state)
    assert action.shape == (2, 3)
    assert log_prob.shape == (2,)
    assert value.shape == (2,)
