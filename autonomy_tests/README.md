# autonomy_tests

ROS 2 测试元包集合。

| 包 | 说明 |
|----|------|
| `autonomy_tests` | metapackage；`ros2 launch autonomy_tests controller_sim.launch.py` 启动仿真 RViz |
| `autonomy_controller` | `autonomy::control` gtest 套件 + controller_sim |
| `autonomy_planner` | `autonomy::planning` gtest 套件 + planner_sim |

```bash
colcon build --packages-up-to autonomy_tests --symlink-install
source install/setup.bash

# 控制器仿真
ros2 launch autonomy_tests controller_sim.launch.py

# 规划器仿真
ros2 launch autonomy_planner planner_sim.launch.py

colcon test --packages-select autonomy_controller autonomy_planner --event-handlers console_direct+
```
