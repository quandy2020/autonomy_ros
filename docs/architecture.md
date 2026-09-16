# 架构：进程隔离

```mermaid
flowchart LR
  subgraph AutonomyProc["autonomy 进程"]
    Core[Navigator / Planner / Controller]
  end
  subgraph RosProc["autonomy_ros 进程"]
    Bridge[AutolinkBridge]
    API[RvizTools]
    VZ[Visualizer]
  end
  RViz[RViz2 / ros2 action]

  Core <-->|autolink channels / actions| Bridge
  Bridge --> VZ --> RViz
  RViz --> API -->|NavigateToPose Action| Core
```

| 模块 | 职责 |
|------|------|
| `AutolinkBridge` | autonomy→ROS：`/map` `/plan` `/cmd_vel` `/odom`（传感器由 autodriver 直进 autonomy） |
| `RvizTools` | ROS Action/Service → autolink Action Client |
| `Visualizer` | ROS 可视化话题 |
| `msgs/` | Nav2 风格外部接口定义 |
