"""Isaac Lab sensor prim paths and observation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import torch
from isaaclab.sensors import ImuCfg, TiledCameraCfg
from isaaclab.utils import configclass

from autonomy_navrl.core.spec import RobotSpec, SensorSpec


@dataclass
class SensorPaths:
    """Resolved USD prim paths for one robot instance."""

    camera: str
    imu: str
    camera_offset_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    camera_offset_rot: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    imu_offset_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)


def resolve_sensor_paths(
    robot: RobotSpec,
    *,
    merge_fixed_joints: bool,
) -> SensorPaths:
    """Build camera / IMU prim paths from robot link layout."""
    prim = robot.prim_name
    if merge_fixed_joints:
        base = robot.base_link
        cam_off = robot.camera_offset or (0.12, 0.0, 0.08)
        cam_rpy = robot.camera_optical_rpy or (-1.5707963268, 0.0, -1.5707963268)
        cy, sy = math.cos(cam_rpy[2] * 0.5), math.sin(cam_rpy[2] * 0.5)
        cp, sp = math.cos(cam_rpy[1] * 0.5), math.sin(cam_rpy[1] * 0.5)
        cr, sr = math.cos(cam_rpy[0] * 0.5), math.sin(cam_rpy[0] * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        imu_off = robot.imu_offset or (0.0, 0.0, 0.0)
        return SensorPaths(
            camera=f'/World/envs/env_.*/{prim}/{base}/rgbd_camera',
            imu=f'/World/envs/env_.*/{prim}/{base}',
            camera_offset_pos=tuple(float(v) for v in cam_off[:3]),
            camera_offset_rot=(qw, qx, qy, qz),
            imu_offset_pos=tuple(float(v) for v in imu_off[:3]),
        )
    return SensorPaths(
        camera=f'/World/envs/env_.*/{prim}/{robot.camera_optical_frame}/rgbd_camera',
        imu=f'/World/envs/env_.*/{prim}/{robot.imu_link}',
    )


def build_tiled_camera_cfg(
    sensor: SensorSpec,
    paths: SensorPaths,
) -> TiledCameraCfg:
    import isaaclab.sim as sim_utils

    horizontal_aperture = 2.0 * sensor.focal_length * math.tan(math.radians(sensor.fov_deg) / 2.0)
    return TiledCameraCfg(
        prim_path=paths.camera,
        offset=TiledCameraCfg.OffsetCfg(
            pos=paths.camera_offset_pos,
            rot=paths.camera_offset_rot,
            convention='ros',
        ),
        data_types=['rgb', 'distance_to_image_plane'],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=sensor.focal_length,
            focus_distance=400.0,
            horizontal_aperture=horizontal_aperture,
            clipping_range=(sensor.depth_min_m, sensor.depth_max_m),
        ),
        width=sensor.rgb_width,
        height=sensor.rgb_height,
        update_period=0.0,
    )


def build_imu_cfg(sensor: SensorSpec, paths: SensorPaths) -> ImuCfg | None:
    if not sensor.imu_enabled:
        return None
    return ImuCfg(
        prim_path=paths.imu,
        offset=ImuCfg.OffsetCfg(pos=paths.imu_offset_pos),
        update_period=0.0,
    )


def rgbd_tensor(camera_output: dict[str, torch.Tensor], max_depth_m: float) -> torch.Tensor:
    """Stack RGB + normalized depth as (N, C, H, W)."""
    rgb = camera_output['rgb']
    depth = camera_output['distance_to_image_plane']
    if rgb.ndim == 4 and rgb.shape[-1] in (3, 4):
        rgb_chw = rgb.float().permute(0, 3, 1, 2) / 255.0
    else:
        rgb_chw = rgb.float() / 255.0
    depth = depth.float()
    if depth.ndim == 4 and depth.shape[-1] == 1:
        depth = depth.squeeze(-1).unsqueeze(1)
    elif depth.ndim == 3:
        depth = depth.unsqueeze(1)
    depth = torch.clamp(depth, 0.0, max_depth_m) / max_depth_m
    return torch.cat([rgb_chw, depth], dim=1)


def odom_features(robot_data, env_origins, pos_xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    return torch.cat([
        pos_xy,
        yaw.unsqueeze(-1),
        robot_data.root_lin_vel_b[:, :2],
        robot_data.root_ang_vel_b[:, 2:3],
    ], dim=-1)


def imu_features(imu_data) -> torch.Tensor:
    return torch.cat([
        imu_data.ang_vel_b,
        imu_data.lin_acc_b,
        imu_data.projected_gravity_b,
    ], dim=-1)
