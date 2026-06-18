# autonomy_pedestrian

Merged ROS 2 package combining:

- **asr_rapidxml** → `third_party/rapidxml/` (header-only XML parser)
- **pedsim_original** → `include/pedsim_original/`, `src/pedsim/`
- **pedestrian_simulator** → `include/pedestrian_simulator/`, `src/simulator/`, `scenarios/`

Inspired by [pedsim_ros2](https://github.com/lehoangan2906/pedsim_ros2): bundled RViz config, `pedsim_rviz_plugin` TrackedPersons display, pause/unpause services, wall/prediction markers, optional pedestrian TF.

## Related packages (sibling under `autonomy_tools/`)

| Package | Role |
|---------|------|
| `pedsim_msgs` | `TrackedPerson` / `TrackedPersons` + trajectory prediction (`ObstacleArray`, `ObstacleGMM`, `Gaussian`) |
| `pedsim_rviz_plugin` | RViz2 displays: TrackedPersons, TrackedGroups, SocialRelations, SocialActivities |
| `ros_tools` | Logging, markers, `walking.dae` mesh for Person meshes style |

## External dependencies

- `derived_object_msgs`, `Eigen3`, `tf2_ros`, `rviz2`, `qtbase5-dev` (build plugin), `pedsim_msgs`

Trajectory prediction messages are in `pedsim_msgs` (`ObstacleArray`, `ObstacleGMM`, `Gaussian`).

## Build

```bash
colcon build --packages-up-to autonomy_pedestrian --allow-overriding autonomy_pedestrian
source install/setup.bash
```

## Launch

```bash
# Corridor demo with RViz + pedsim_rviz_plugin (default)
ros2 launch autonomy_pedestrian pedestrian.launch.py \
  pedestrian_scenario:=random_social/8_corridor.xml

# MarkerArray-only visualization (no plugin bridge)
ros2 launch autonomy_pedestrian pedestrian.launch.py use_pedestrian_rviz_plugin:=false

# Keyboard-controlled robot pose
ros2 launch autonomy_pedestrian pedestrian.launch.py \
  run_demo_helpers:=false use_keyboard_teleop:=true

# Publish pedestrian TF frames (map -> pedestrian_<id>)
ros2 launch autonomy_pedestrian pedestrian.launch.py publish_pedestrian_tf:=true
```

### Topics

| Topic | Type |
|-------|------|
| `/pedestrian_simulator/pedestrians` | `derived_object_msgs/ObjectArray` |
| `/pedestrian_simulator/trajectory_predictions` | `pedsim_msgs/ObstacleArray` |
| `/pedestrian_simulator/visualization` | `visualization_msgs/MarkerArray` |
| `/pedestrian_simulator/walls` | `visualization_msgs/MarkerArray` |
| `/pedestrian_simulator/prediction_paths` | `visualization_msgs/MarkerArray` |
| `/pedestrian_simulator/static_obstacles` | `visualization_msgs/MarkerArray` |
| `/pedestrian_simulator/robot_position` | `visualization_msgs/MarkerArray` |
| `/pedestrian_visualizer/tracked_persons` | `pedsim_msgs/TrackedPersons` |
| `/pedestrian_visualizer/tracked_groups` | `pedsim_msgs/TrackedGroups` |
| `/pedestrian_visualizer/social_relations` | `pedsim_msgs/SocialRelations` |
| `/pedestrian_visualizer/social_activities` | `pedsim_msgs/SocialActivities` |
| `/robot_state` | `geometry_msgs/PoseStamped` (demo / teleop) |

### Services

| Service | Description |
|---------|-------------|
| `/pedestrian_simulator/start` | Start simulation loop |
| `/pedestrian_simulator/pause_simulation` | Pause updates |
| `/pedestrian_simulator/unpause_simulation` | Resume updates |

Executable: `pedestrian_simulator_node`  
Scenarios: `share/autonomy_pedestrian/scenarios/`  
RViz: `share/autonomy_pedestrian/rviz/pedestrian.rviz`
