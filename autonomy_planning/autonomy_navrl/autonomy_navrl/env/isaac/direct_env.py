"""Isaac Lab DirectRLEnv — multi-robot visual navigation."""

from __future__ import annotations

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import Imu, TiledCamera

from autonomy_navrl.core.sampling import sample_obstacle_positions
from autonomy_navrl.env.isaac.cfg import NavrlEnvCfg
from autonomy_navrl.plugins import bootstrap as _bootstrap
from autonomy_navrl.plugins.actions import create_action_adapter
from autonomy_navrl.plugins.locomotion.base import create_locomotion_controller
from autonomy_navrl.plugins.rewards import RewardStepContext, create_reward_computer
from autonomy_navrl.plugins.jdrobot.runtime import build_goal_curriculum, create_action_smoother
from autonomy_navrl.plugins.isaac_sensors import imu_features, odom_features, rgbd_tensor
from autonomy_navrl.plugins.tasks import create_task
from autonomy_navrl.viz.isaac.context import PoseVizContext
from autonomy_navrl.viz.isaac.manager import build_pose_viz_manager

_bootstrap.ensure_plugins()


class NavrlDirectEnv(DirectRLEnv):
    """Visual navigation with pluggable robot, task, action, and sensors."""

    cfg: NavrlEnvCfg

    def __init__(self, cfg: NavrlEnvCfg, render_mode: str | None = None, **kwargs) -> None:
        self._goal_rng = None
        super().__init__(cfg, render_mode, **kwargs)
        self._goals = torch.zeros(self.num_envs, 2, device=self.device)
        self._goal_yaws = torch.zeros(self.num_envs, device=self.device)
        self._prev_distance = torch.zeros(self.num_envs, device=self.device)
        action_dim = int(cfg.action_space)
        self._prev_actions = torch.zeros(self.num_envs, action_dim, device=self.device)
        self._reward_profile = getattr(cfg, 'reward_profile', 'default')

        control_dt = float(cfg.sim.dt) * int(cfg.decimation)
        self._action_smoother = create_action_smoother(
            self.num_envs,
            self.device,
            getattr(cfg, '_action_smoothing_cfg', None),
            control_dt,
        )
        self._goal_curriculum = getattr(
            cfg,
            '_goal_curriculum',
            build_goal_curriculum({'goal_distance_m': cfg.goal_distance_m}),
        )
        self._reward_computer = create_reward_computer(self._reward_profile, self)
        self._pose_viz_manager = build_pose_viz_manager(getattr(cfg, '_viz_cfg', None), self.device)

        framework = cfg._framework
        assert framework is not None
        self._action_adapter = create_action_adapter(
            framework.action, self.num_envs, self.device
        )
        self._task = create_task(framework.task, self)

    def _setup_scene(self) -> None:
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        self._robot = Articulation(self.cfg.robot_cfg)
        self._camera = TiledCamera(self.cfg.tiled_camera)
        self._imu = Imu(self.cfg.imu) if self.cfg.imu is not None else None

        copy_from_source = not self.scene.cfg.replicate_physics
        self.scene.clone_environments(copy_from_source=copy_from_source)
        if not self.scene.cfg.replicate_physics:
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        self.scene.articulations['robot'] = self._robot
        self.scene.sensors['tiled_camera'] = self._camera
        if self._imu is not None:
            self.scene.sensors['imu'] = self._imu

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func('/World/Light', light_cfg)
        self._spawn_obstacles()
        loc_spec = getattr(self.cfg, '_locomotion_spec', None)
        if loc_spec is None and self.cfg._framework is not None:
            loc_spec = self.cfg._framework.locomotion
        from autonomy_navrl.core.locomotion_spec import LocomotionSpec

        self._locomotion = create_locomotion_controller(
            loc_spec or LocomotionSpec(), self
        )
        print(
            f'[INFO]: Locomotion backend: {self._locomotion.name}',
            flush=True,
        )

    def _spawn_obstacles(self) -> None:
        if self.cfg.num_obstacles <= 0:
            return
        positions = sample_obstacle_positions(
            self.cfg.num_obstacles,
            self.cfg.arena_size_m,
            min_separation_m=self.cfg.obstacle_size_m * 2.0,
            rng=self._numpy_rng(),
        )
        half_h = self.cfg.obstacle_height_m * 0.5
        size = self.cfg.obstacle_size_m
        obstacle_cfg = sim_utils.CuboidCfg(
            size=(size, size, self.cfg.obstacle_height_m),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.35, 0.2)),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
        )
        for index, pos in enumerate(positions):
            obstacle_cfg.func(
                f'/World/envs/env_.*/obstacle_{index}',
                obstacle_cfg,
                translation=(float(pos[0]), float(pos[1]), half_h),
            )

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._action_adapter.pre_physics_step(actions)
        if self._action_smoother is not None and hasattr(self._action_adapter, 'apply_smoothing'):
            vx = getattr(self._action_adapter, '_vx')
            vy = getattr(self._action_adapter, '_vy')
            w = getattr(self._action_adapter, '_w')
            vx, vy, w = self._action_smoother.apply(vx, vy, w)
            self._action_adapter.apply_smoothing(vx, vy, w)

    def _apply_action(self) -> None:
        assert self._locomotion is not None
        self._locomotion.apply()

    def _get_observations(self) -> dict:
        rgbd = rgbd_tensor(self._camera.data.output, self.cfg.max_depth_m)
        state = self._build_state_vector()
        return {'rgbd': rgbd, 'state': state, 'sensor_backend': 'isaaclab'}

    def _depth_map(self) -> torch.Tensor:
        depth = self._camera.data.output['distance_to_image_plane'].float()
        if depth.ndim == 4 and depth.shape[-1] == 1:
            depth = depth.squeeze(-1)
        elif depth.ndim == 4 and depth.shape[1] == 1:
            depth = depth.squeeze(1)
        return depth

    def _get_rewards(self) -> torch.Tensor:
        pos_xy = self._robot_pos_xy()
        distance = torch.linalg.norm(self._goals - pos_xy, dim=-1)
        progress = self._prev_distance - distance
        depth = self._depth_map()
        proximity = (depth < self.cfg.collision_threshold_m).float().mean(dim=(1, 2))
        yaw = self._robot_yaw()
        yaw_error = self._yaw_error()
        goal_delta = self._goals - pos_xy
        bearing = torch.atan2(goal_delta[:, 1], goal_delta[:, 0]) - yaw
        bearing = torch.atan2(torch.sin(bearing), torch.cos(bearing))
        reached = self._task.is_success(distance, yaw_error)

        self._task.on_step(pos_xy, yaw)
        commands = self._action_adapter.commands
        lin_speed_b = torch.linalg.norm(self._robot.data.root_lin_vel_b[:, :2], dim=-1)

        ctx = RewardStepContext(
            distance=distance,
            progress=progress,
            proximity=proximity,
            bearing=bearing,
            yaw_error=yaw_error,
            reached=reached,
            commands=commands,
            prev_commands=self._prev_actions,
            lin_speed_b=lin_speed_b,
            collision_terminated=proximity > 0.35,
            timeout=self.reset_time_outs,
        )
        reward, metrics = self._reward_computer.compute(ctx)

        metrics['reward_profile'] = self._reward_profile
        metrics['task_kind'] = self.cfg.task_kind
        metrics['robot_kind'] = self.cfg._framework.robot.kind if self.cfg._framework else 'unknown'
        if self._locomotion is not None:
            metrics['locomotion'] = self._locomotion.name
        if self.cfg.task_kind in ('precision_pose', 'go2w_precision_pose'):
            metrics['yaw_error_mean'] = float(yaw_error.abs().mean().detach().cpu())
            metrics.update(self._task.reward_metrics_extra())
        self.extras['metrics'] = metrics
        self._prev_distance = distance.detach()
        self._prev_actions = commands.detach()
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        pos_xy = self._robot_pos_xy()
        distance = torch.linalg.norm(self._goals - pos_xy, dim=-1)
        depth = self._depth_map()
        proximity = (depth < self.cfg.collision_threshold_m).float().mean(dim=(1, 2))
        collision_hit = proximity > 0.35
        if not getattr(self.cfg, 'collision_terminate', True):
            collision_hit = torch.zeros_like(collision_hit, dtype=torch.bool)
        terminated = self._task.is_success(distance, self._yaw_error()) | collision_hit
        truncated = self.episode_length_buf >= self.max_episode_length - 1
        return terminated, truncated

    def _reset_idx(self, env_ids: torch.Tensor | None) -> None:
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        super()._reset_idx(env_ids)
        self._task.reset_goals(env_ids)
        pos_xy = self._robot_pos_xy()[env_ids]
        self._prev_distance[env_ids] = torch.linalg.norm(self._goals[env_ids] - pos_xy, dim=-1)
        self._prev_actions[env_ids] = 0.0
        if self._action_smoother is not None:
            self._action_smoother.reset(env_ids)
        self._reward_computer.reset(env_ids)
        self._task.on_reset_envs(env_ids)
        # Joints + upright root first, then locomotion warmup (matches deploy needs).
        self._reset_robot_pose_and_joints(env_ids)
        if self._locomotion is not None:
            self._locomotion.reset(env_ids)

    def _reset_robot_pose_and_joints(self, env_ids: torch.Tensor) -> None:
        """Force standing joint pose and upright root (zero vel) after episode reset."""
        from isaaclab.utils.math import quat_from_euler_xyz

        count = len(env_ids)
        spawn_z = float(self.cfg.robot_cfg.init_state.pos[2])
        root_pose = self._robot.data.root_state_w[env_ids, :7].clone()
        # Keep XY from scene reset; restore height and upright orientation.
        root_pose[:, 2] = spawn_z + self.scene.env_origins[env_ids, 2]
        yaw = self._robot_yaw()[env_ids]
        zeros = torch.zeros_like(yaw)
        root_pose[:, 3:7] = quat_from_euler_xyz(zeros, zeros, yaw)
        self._robot.write_root_pose_to_sim(root_pose, env_ids=env_ids)
        self._robot.write_root_com_velocity_to_sim(
            torch.zeros(count, 6, device=self.device), env_ids=env_ids,
        )
        self._reset_robot_joints(env_ids)

    def _reset_robot_joints(self, env_ids: torch.Tensor) -> None:
        init_joint_pos = getattr(self.cfg, '_init_joint_pos', None)
        if not init_joint_pos:
            return
        joint_pos = self._robot.data.default_joint_pos[env_ids].clone()
        joint_vel = torch.zeros_like(joint_pos)
        for name, value in init_joint_pos.items():
            joint_ids, _ = self._robot.find_joints(name)
            if len(joint_ids) == 0:
                continue
            joint_pos[:, joint_ids] = float(value)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

    def set_robot_spawn(
        self,
        env_ids: torch.Tensor,
        x: float,
        y: float,
        yaw: float,
        z: float | None = None,
    ) -> None:
        """Set robot root pose in env-local XY (m) and yaw (rad), then reset joints/locomotion."""
        from isaaclab.utils.math import quat_from_euler_xyz

        count = len(env_ids)
        spawn_z = float(z if z is not None else self.cfg.robot_cfg.init_state.pos[2])
        pos_w = torch.zeros(count, 3, device=self.device)
        pos_w[:, 0] = float(x) + self.scene.env_origins[env_ids, 0]
        pos_w[:, 1] = float(y) + self.scene.env_origins[env_ids, 1]
        pos_w[:, 2] = spawn_z + self.scene.env_origins[env_ids, 2]
        yaw_t = torch.full((count,), float(yaw), device=self.device)
        zeros = torch.zeros_like(yaw_t)
        quat = quat_from_euler_xyz(zeros, zeros, yaw_t)
        self._robot.write_root_pose_to_sim(torch.cat([pos_w, quat], dim=-1), env_ids=env_ids)
        self._robot.write_root_com_velocity_to_sim(
            torch.zeros(count, 6, device=self.device),
            env_ids=env_ids,
        )
        self._reset_robot_joints(env_ids)
        if self._locomotion is not None:
            self._locomotion.reset(env_ids)
        pos_xy = self._robot_pos_xy()[env_ids]
        self._prev_distance[env_ids] = torch.linalg.norm(
            self._goals[env_ids] - pos_xy, dim=-1,
        )

    def _build_state_vector(self) -> torch.Tensor:
        pos_xy = self._robot_pos_xy()
        yaw = self._robot_yaw()
        goal_feat = self._task.goal_state_features(pos_xy, yaw)
        parts = [goal_feat]
        framework = self.cfg._framework
        if self._imu is not None and framework and framework.sensors.imu_enabled:
            parts.append(imu_features(self._imu.data))
        if framework and framework.sensors.odom_enabled:
            parts.append(odom_features(self._robot.data, self.scene.env_origins, pos_xy, yaw))
        packed = torch.cat(parts, dim=-1)
        state = torch.zeros(self.num_envs, self.cfg.state_dim, device=self.device)
        copy_len = min(self.cfg.state_dim, packed.shape[1])
        state[:, :copy_len] = packed[:, :copy_len]
        return state

    def _robot_pos_xy(self) -> torch.Tensor:
        return self._robot.data.root_pos_w[:, :2] - self.scene.env_origins[:, :2]

    def _robot_yaw(self) -> torch.Tensor:
        quat = self._robot.data.root_quat_w
        w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
        return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _yaw_error(self) -> torch.Tensor:
        error = self._goal_yaws - self._robot_yaw()
        return torch.atan2(torch.sin(error), torch.cos(error))

    def _numpy_rng(self):
        import numpy as np
        if self._goal_rng is None:
            seed = getattr(self.cfg, 'seed', None)
            self._goal_rng = np.random.default_rng(seed)
        return self._goal_rng

    def _footprint_dims(self, viz_cfg) -> tuple[float, float, float, float]:
        """Resolve spot/robot footprint sizes from viz config or task IoU params."""
        task_params = getattr(self.cfg, '_task_params', None) or {}
        iou_cfg = task_params.get('consecutive_iou', {})
        spot_length = float(
            iou_cfg.get('spot_length', getattr(viz_cfg, 'spot_length', 0.9))
        )
        spot_width = float(
            iou_cfg.get('spot_width', getattr(viz_cfg, 'spot_width', 0.3))
        )
        robot_length = float(
            iou_cfg.get('robot_length', getattr(viz_cfg, 'robot_length', 0.9))
        )
        robot_width = float(
            iou_cfg.get('robot_width', getattr(viz_cfg, 'robot_width', 0.3))
        )
        return spot_length, spot_width, robot_length, robot_width

    def _build_pose_viz_context(self) -> PoseVizContext:
        viz_cfg = getattr(self.cfg, '_viz_cfg', None)
        z_offset = float(getattr(viz_cfg, 'marker_z_offset', 0.05))
        # Pin markers to ground plane — do not follow robot body bounce in Z.
        ground_z = self.scene.env_origins[:, 2] + z_offset
        target_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        target_pos_w[:, :2] = self._goals + self.scene.env_origins[:, :2]
        target_pos_w[:, 2] = ground_z
        robot_vis = self._robot.data.root_pos_w.clone()
        robot_vis[:, 2] = ground_z
        spot_length, spot_width, robot_length, robot_width = self._footprint_dims(viz_cfg)
        return PoseVizContext(
            robot_pos_w=robot_vis,
            robot_yaw=self._robot_yaw(),
            target_pos_w=target_pos_w,
            target_yaw=self._goal_yaws,
            goal_tolerance_m=float(self.cfg.goal_tolerance_m),
            commands=self._action_adapter.commands,
            max_vx=float(self.cfg.max_vx),
            max_vy=float(self.cfg.max_vy),
            max_w=float(self.cfg.max_w),
            spot_length=spot_length,
            spot_width=spot_width,
            robot_length=robot_length,
            robot_width=robot_width,
        )

    def _set_debug_vis_impl(self, debug_vis: bool) -> None:
        if self._pose_viz_manager is None:
            return
        self._pose_viz_manager.set_visible(debug_vis)
        if debug_vis:
            self._pose_viz_manager.update(self._build_pose_viz_context())

    def _debug_vis_callback(self, event) -> None:
        if self._pose_viz_manager is None or not hasattr(self, 'scene'):
            return
        self._pose_viz_manager.update(self._build_pose_viz_context())
