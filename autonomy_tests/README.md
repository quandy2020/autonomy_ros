# autonomy_tests

ROS 2 测试元包集合。

| 包 | 说明 |
|----|------|
| `autonomy_tests` | metapackage；`ros2 launch autonomy_tests controller_sim.launch.py` 启动仿真 RViz |
| `autonomy_controller` | `autonomy::control` gtest 套件 + controller_sim |
| `autonomy_planner` | `autonomy::planning` gtest 套件 + planner_sim |
| `autonomy_autocharge` | Nav2 predock + autocharge 回充联调 demo |

```bash
colcon build --packages-up-to autonomy_tests --symlink-install
source install/setup.bash

# 控制器仿真
ros2 launch autonomy_tests controller_sim.launch.py

# 规划器仿真
ros2 launch autonomy_planner planner_sim.launch.py

# 自动回充 demo（Habitat + Nav2 + autocharge）
colcon build --packages-select autonomy_autocharge autocharge jdbot_interfaces --symlink-install
source install/setup.bash
ros2 launch autonomy_autocharge autocharge_demo.launch.py
# 或通过 metapackage 入口：
# ros2 launch autonomy_tests autocharge_demo.launch.py

colcon test --packages-select autonomy_controller autonomy_planner --event-handlers console_direct+
```
