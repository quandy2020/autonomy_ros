#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""ROS 2 parameters for the Habitat bridge."""

from __future__ import annotations

import os
from dataclasses import MISSING, dataclass

from rclpy.node import Node


@dataclass(frozen=True)
class Config:
    """Immutable bridge configuration loaded from ROS parameters."""

    # Required: no defaults; set via param/habitat.yaml or launch.
    scene_data_path: str
    scene_id: str
    mp3d_root: str
    scene_dataset_config: str
    package_scene_dataset_config: str

    # Habitat camera sensor specs (defaults match param/habitat.yaml).
    image_width: int = 1280
    image_height: int = 720
    camera_horizontal_fov_deg: float = 70.0
    sensor_height: float = 0.6  # Habitat agent sensor Y offset.

    agent_pose_topic: str = 'habitat/agent_pose'
    set_agent_pose_topic: str = 'habitat/set_agent_pose'
    agent_pose_frame: str = 'odom'

    # Frame names must match urdf/habitat.urdf and image header.frame_id.
    map_frame: str = 'map'
    odom_frame: str = 'odom'
    base_footprint_frame: str = 'base_footprint'
    base_link_height: float = 0.01
    rgb_camera_frame: str = 'camera_optical_frame'
    depth_camera_frame: str = 'camera_optical_frame'
    semantic_camera_frame: str = 'camera_optical_frame'

    odom_topic: str = 'odom'
    cmd_vel_topic: str = 'cmd_vel'
    cmd_vel_timeout: float = 1.0

    rgb_topic: str = 'camera/rgb/image_raw'
    rgb_camera_info_topic: str = 'camera/rgb/camera_info'
    depth_topic: str = 'camera/depth/image_raw'
    semantic_topic: str = 'camera/semantic/image_raw'

    # topdown_mode: room | oblique | overhead | interactive (RViz 6-DOF marker)
    topdown_enabled: bool = False
    topdown_topic: str = 'camera/topdown/image_raw'
    topdown_camera_info_topic: str = 'camera/topdown/camera_info'
    topdown_frame: str = 'map'
    topdown_mode: str = 'room'
    topdown_width: int = 640
    topdown_height: int = 480
    topdown_height_m: float = 8.0
    topdown_distance_m: float = 7.0
    topdown_look_ahead_m: float = 3.0
    topdown_look_at_height_m: float = 1.2
    topdown_hfov_deg: float = 90.0
    topdown_room_margin: float = 1.18
    topdown_ortho_scale: float = 0.0  # 0 = auto from navmesh; meters (room mode ORTHOGRAPHIC)

    semantic_ply_path: str = ''  # Empty: use {scene_dir}/{scene_id}_semantic.ply.
    semantic_pointcloud_topic: str = 'semantic_pointcloud'
    semantic_pointcloud_frame: str = 'map'
    semantic_pointcloud_downsample: int = 1
    semantic_pointcloud_rate_hz: float = 1.0  # 0 = latched once; <0 = disabled.

    occupancy_grid_topic: str = 'map'
    occupancy_grid_frame: str = 'map'
    occupancy_grid_resolution: float = 0.05
    occupancy_grid_z_min: float = 0.0  # PLY Z-up floor slice (m).
    occupancy_grid_z_max: float = 0.55
    occupancy_grid_downsample: int = 1
    occupancy_grid_rate_hz: float = 1.0  # 0 = latched once; <0 = disabled.

    navmesh_topic: str = 'habitat/graph'
    navmesh_mesh_topic: str = 'habitat/navmesh_mesh'
    navmesh_frame: str = 'map'
    navmesh_topology: str = 'triangle'  # triangle | polygon | grid
    navmesh_rate_hz: float = 0.0  # 0 = latched once; <0 = disabled.
    navmesh_meters_per_pixel: float = 0.4
    navmesh_height: float = 0.0  # 0 = use agent floor height.
    navmesh_eps: float = 0.5
    navmesh_stride: int = 2
    navmesh_z: float = 0.05
    navmesh_max_nodes: int = 20000
    navmesh_node_scale: float = 0.08
    navmesh_line_width: float = 0.03
    navmesh_show_nodes: bool = True
    navmesh_show_edges: bool = True
    navmesh_show_mesh: bool = False
    navmesh_show_faces: bool = True
    navmesh_face_alpha: float = 0.18
    navmesh_color_by_island: bool = True
    navmesh_connect_diagonal: bool = False

    update_rate_hz: float = 20.0  # Sim step + camera/odom publish rate.

    # Multi-robot spawn: each Habitat instance picks a dispersed navmesh point.
    spawn_mode: str = 'dispersed'  # dispersed | random | fixed
    spawn_index: int = 0
    spawn_count: int = 1
    spawn_seed: int = 0  # 0 = derive from scene_id
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_yaw: float = 0.0
    # Min navmesh distance to walls/obstacles (≈ robot_radius + costmap inflation).
    spawn_clearance_m: float = 0.50

    # Dynamic pedestrians (evt_bench-style oracle nav on navmesh).
    pedestrians_enabled: bool = False
    pedestrian_count: int = 5
    pedestrian_goal_count: int = 4
    pedestrian_linear_speed: float = 0.5
    pedestrian_angular_speed: float = 1.5
    pedestrian_dist_thresh: float = 0.5
    pedestrian_turn_thresh: float = 0.3
    pedestrian_waypoint_min_dist: float = 3.0
    pedestrian_robot_activate_dist: float = -1.0
    pedestrian_avoid_dist: float = 2.0
    pedestrian_spawn_min_robot_dist: float = 2.5
    robot_avoid_radius: float = 0.35
    pedestrian_robot_avoid_dist: float = 0.0  # 0 = only nudge when nearly touching
    pedestrian_radius: float = 0.35
    pedestrian_height: float = 1.7
    pedestrian_seed: int = 0
    pedestrians_tracked_topic: str = 'pedestrian_visualizer/tracked_persons'
    pedestrians_viz_topic: str = 'pedestrian_simulator/visualization'
    trackvla_root: str = ''
    humanoid_data_root: str = ''
    humanoid_avatar: str = 'female_2'
    # Comma-separated avatar names; empty = auto-discover all under humanoid_data_root
    humanoid_avatars: str = ''
    humanoid_infos_json: str = ''
    pedestrian_semantic_id: int = 250
    dynamic_actor_kind: str = 'humanoid'  # humanoid | robot
    human_agent_count: int = 0
    human_agent_goal_count: int = 4
    human_agent_linear_speed: float = 0.5
    human_agent_seed: int = 0
    human_agents_tracked_topic: str = 'pedestrian_visualizer/tracked_persons'
    human_agents_viz_topic: str = 'pedestrian_simulator/visualization'
    robot_asset_root: str = ''
    robot_asset_type: str = 'turtlebot3_waffle'
    # Comma-separated robot asset names; empty = auto-discover all under robot_asset_root
    robot_asset_types: str = ''
    # Comma-separated exact robot counts, e.g. "spot=2,jackal=1"
    robot_asset_counts: str = ''
    robot_semantic_id: int = 251
    robot_agent_count: int = 0
    robot_agent_goal_count: int = 4
    robot_agent_linear_speed: float = 0.5
    robot_agent_seed: int = 0
    robot_agents_tracked_topic: str = 'robot_agent_visualizer/tracked_persons'
    robot_agents_viz_topic: str = 'robot_simulator/visualization'
    robot_agents_debug_mesh_viz: bool = False
    # Comma-separated overrides, e.g. "jackal=0.32,husky=0.45"
    robot_radius_overrides: str = ''
    robot_height_overrides: str = ''
    robot_semantic_id_overrides: str = ''

    def dataset_config(self) -> str:
        if self.scene_dataset_config:
            return self.scene_dataset_config
        return os.path.join(self.mp3d_root, 'mp3d.scene_dataset_config.json')

    def scene_asset(self) -> str:
        # Habitat-Sim scene_id relative to mp3d.scene_dataset_config.json.
        return f'{self.scene_id}/{self.scene_id}.glb'

    def scene_dir(self) -> str:
        if self.scene_data_path:
            return self.scene_data_path.rstrip('/')
        return os.path.join(self.mp3d_root, self.scene_id)


def load(node: Node) -> Config:
    fields = Config.__dataclass_fields__

    # Params from --params-file are overrides; they apply when each param is declared.
    to_declare = [
        (name, '' if field.default is MISSING else field.default)
        for name, field in fields.items()
        if not node.has_parameter(name)
    ]
    if to_declare:
        node.declare_parameters('', to_declare)

    values = {name: node.get_parameter(name).value for name in fields}
    path = str(values['scene_data_path']).rstrip('/')
    if not path:
        from autonomy_lerobot.data_paths import default_mp3d_scene

        path = str(default_mp3d_scene())
        values['scene_data_path'] = path
    if path:
        # scene_data_path overrides scene_id and mp3d_root from launch/yaml.
        values['scene_id'] = os.path.basename(path)
        values['mp3d_root'] = os.path.dirname(path)
        values['scene_data_path'] = path
    elif not str(values['scene_id']).strip() or not str(values['mp3d_root']).strip():
        raise RuntimeError(
            'Missing scene path: set scene_data_path or both scene_id and mp3d_root '
            'in param/habitat.yaml or launch'
        )

    if not str(values['package_scene_dataset_config']).strip():
        raise RuntimeError(
            'Missing package_scene_dataset_config (set in launch or param/habitat.yaml)'
        )

    values['spawn_index'] = int(values['spawn_index'])
    values['spawn_count'] = max(1, int(values['spawn_count']))
    values['spawn_seed'] = int(values['spawn_seed'])
    values['spawn_x'] = float(values['spawn_x'])
    values['spawn_y'] = float(values['spawn_y'])
    values['spawn_yaw'] = float(values['spawn_yaw'])
    values['spawn_clearance_m'] = max(0.0, float(values['spawn_clearance_m']))
    values['spawn_mode'] = str(values['spawn_mode']).strip().lower() or 'fixed'
    values['pedestrian_count'] = max(0, int(values['pedestrian_count']))
    values['pedestrian_goal_count'] = max(1, int(values['pedestrian_goal_count']))
    values['pedestrian_seed'] = int(values['pedestrian_seed'])
    values['pedestrian_semantic_id'] = int(values['pedestrian_semantic_id'])
    values['dynamic_actor_kind'] = str(values['dynamic_actor_kind']).strip().lower() or 'humanoid'
    if values['dynamic_actor_kind'] not in ('humanoid', 'robot'):
        values['dynamic_actor_kind'] = 'humanoid'
    values['robot_semantic_id'] = int(values['robot_semantic_id'])
    values['human_agent_count'] = max(0, int(values['human_agent_count']))
    values['human_agent_goal_count'] = max(1, int(values['human_agent_goal_count']))
    values['human_agent_seed'] = int(values['human_agent_seed'])
    values['robot_agent_count'] = max(0, int(values['robot_agent_count']))
    values['robot_agent_goal_count'] = max(1, int(values['robot_agent_goal_count']))
    values['robot_agent_seed'] = int(values['robot_agent_seed'])
    if values['human_agent_count'] == 0 and values['robot_agent_count'] == 0:
        if bool(values['pedestrians_enabled']) and values['pedestrian_count'] > 0:
            if values['dynamic_actor_kind'] == 'robot':
                values['robot_agent_count'] = int(values['pedestrian_count'])
                values['robot_agent_goal_count'] = int(values['pedestrian_goal_count'])
                values['robot_agent_linear_speed'] = float(values['pedestrian_linear_speed'])
                values['robot_agent_seed'] = int(values['pedestrian_seed'])
            else:
                values['human_agent_count'] = int(values['pedestrian_count'])
                values['human_agent_goal_count'] = int(values['pedestrian_goal_count'])
                values['human_agent_linear_speed'] = float(values['pedestrian_linear_speed'])
                values['human_agent_seed'] = int(values['pedestrian_seed'])
    values['pedestrians_enabled'] = bool(
        values['pedestrians_enabled']
        or values['human_agent_count'] > 0
        or values['robot_agent_count'] > 0
    )
    values['topdown_enabled'] = bool(values['topdown_enabled'])
    values['topdown_mode'] = str(values['topdown_mode']).strip().lower() or 'room'
    if values['topdown_mode'] not in ('room', 'oblique', 'overhead', 'interactive'):
        values['topdown_mode'] = 'room'
    values['topdown_width'] = max(64, int(values['topdown_width']))
    values['topdown_height'] = max(64, int(values['topdown_height']))
    values['topdown_height_m'] = max(1.0, float(values['topdown_height_m']))
    values['topdown_distance_m'] = max(0.5, float(values['topdown_distance_m']))
    values['topdown_look_ahead_m'] = float(values['topdown_look_ahead_m'])
    values['topdown_look_at_height_m'] = max(0.0, float(values['topdown_look_at_height_m']))
    values['topdown_hfov_deg'] = float(values['topdown_hfov_deg'])
    values['topdown_room_margin'] = max(1.0, float(values['topdown_room_margin']))
    values['topdown_ortho_scale'] = max(0.0, float(values['topdown_ortho_scale']))
    if not str(values['humanoid_data_root']).strip():
        try:
            from autonomy_lerobot.data_paths import humanoid_data_root

            values['humanoid_data_root'] = str(humanoid_data_root())
        except Exception:
            pass

    return Config(**values)
