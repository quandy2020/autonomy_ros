# autonomy_ros

A **ROS 2 autonomous navigation** workspace focused on two production features: **single-goal navigation** and **multi-waypoint navigation**.

It combines the `autonomy` navigation core, the `autonomy_ros` ROS integration layer

*  `autonomy_msgs` API package
* `autonomy_simulator` 
*  simulation assets (based on [nav2_minimal_tb3_sim](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation/tree/main/nav2_minimal_tb3_sim)).

---

## ✨ Features

- 🎯 **Single-goal navigation** via `/navigate_pose` action and RViz `goal_pose`.
- 🧭 **Multi-waypoint navigation** via `/navigate_through` action with waypoint sequencing.
- 🛠️ **Task control APIs** for cancel, pause/resume, emergency stop, and status query.
- 🧪 **Simulation-ready workflow** with Gazebo and fake-robot modes in one launch entry.

## 🎥 Demo

- Video file: [`autonomy-sim.mp4`](autonomy_ros/docs/autonomy-sim.mp4)

<video controls width="960">
  <source src="autonomy_ros/docs/autonomy-sim.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

---

## 📦 Dependencies and build

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-ros-gz-interfaces \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-xacro

cd /path/to/your_ws
colcon build --symlink-install --packages-up-to autonomy autonomy_simulator autonomy_ros
source install/setup.bash
```

---

## 🚀 Launch

Use `navigation_stack.launch.py` for the navigation stack, and control behavior with launch arguments.

```bash
ros2 launch autonomy_ros navigation_stack.launch.py
```

### 🧩 Common scenarios

1) **Full stack with RViz**

```bash
ros2 launch autonomy_ros navigation_stack.launch.py use_rviz:=true
```

2) **Fake robot + navigation stack**

```bash
ros2 launch autonomy_ros navigation_stack.launch.py \
  simulation_mode:=fake use_sim_time:=false
```

For fake mode, set `autonomy.enable_scan_bridge=false` in `config/parameters.yaml` and keep `autonomy.planner.global_frame=odom`.

3) **Simulation only (no navigation stack)**

```bash
ros2 launch autonomy_simulator tb3_simulator.launch.py
ros2 launch autonomy_simulator fake_robot.launch.py
```

### ⚙️ Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `simulation_mode` | `gazebo` | `gazebo` or `fake` |
| `use_sim_time` | `true` | Use simulation clock |
| `use_rviz` | `false` | Start RViz |
| `core_config_directory` | auto-resolved | Directory containing `autonomy.lua` |

---

## 🔌 Public API

### 🎬 Actions

| Name | Description |
|------|-------------|
| `/navigate_pose` | Single-goal navigation |
| `/navigate_through` | Sequential multi-waypoint navigation |

### 🧰 Services

| Name | Description |
|------|-------------|
| `/cancel_task` | Cancel a task |
| `/get_task_status` | Query task status |
| `/pause_task` / `/resume_task` | Pause / resume |
| `/trigger_estop` | Emergency stop |
| `/set_initial_pose` | Set initial pose (localization) |

### 📡 Status topics

| Name | Type |
|------|------|
| `/status` | `autonomy_msgs/msg/TaskStatus` |
| `/events` | `autonomy_msgs/msg/Event` |

### 💻 CLI examples

```bash
ros2 run autonomy_ros navigation_client.py list-tasks
ros2 run autonomy_ros navigation_client.py run-task go_to_point_a --feedback
ros2 run autonomy_ros navigation_client.py navigate-pose --x 1.0 --y 0.5 --feedback
```

### 🖼️ RViz

- **2D Goal Pose** → `goal_pose`: triggers single-goal navigation (may preempt the current action)
- **2D Pose Estimate** → `initialpose`: forwarded by `NavigationService` for relocalization

---

## 🙏 Acknowledgements

- Simulation: [nav2_minimal_turtlebot_simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation) (Apache-2.0)
- API inspiration: [nav2_msgs](https://github.com/ros-navigation/navigation2/tree/main/nav2_msgs)
