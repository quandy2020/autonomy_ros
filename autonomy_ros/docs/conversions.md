# 消息转换（conversions）

`autonomy_ros` 在 ROS 2 `*::msg::*` 与 `automsgs::msgs::*` protobuf 之间做**逐字段拷贝**，不做 TF 变换或时间重映射。

- **提供**：`fromRos`、`toRos`
- 核心侧直接使用 automsgs protobuf 类型，不再经过 `ToProto` / `FromProto` 兼容层

---

## 用法

```cpp
#include "autonomy_ros/conversions/conversions.hpp"

void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto core_odom = autonomy_ros::fromRos(*msg);
  // ...
}

plan_pub_->publish(autonomy_ros::toRos(path));
```

可按需只 include 子模块（如 `geometry_msgs.hpp`）以减少编译依赖。

---

## 当前实现的模块

导航栈实际编译并使用的转换模块：

```
include/autonomy_ros/conversions/
  conversions.hpp          # 总入口
  builtin_interfaces.hpp
  std_msgs.hpp
  geometry_msgs.hpp
  sensor_msgs.hpp
  map_msgs.hpp             # OccupancyGrid 等（含 octomap/grid_map 段，按需使用）
  planning_msgs.hpp        # Path、Odometry（映射到 automsgs nav_msgs）
  tf2_msgs.hpp
  detail.hpp               # 内部字段拷贝
```

| 模块 | 典型调用方 |
|------|------------|
| `geometry_msgs` | 导航 goal、cmd_vel、位姿 |
| `sensor_msgs` | `RosBridge`（LaserScan、PointCloud2、Range） |
| `map_msgs` / `planning_msgs` | `RosBridge`、`TaskManager`、`Visualizer` |
| `tf2_msgs` | `RosBridge` |
| `nav_msgs::Path` | `TaskManager`、`Visualizer` |

---

## 在架构中的位置

```mermaid
flowchart LR
  ROS["ROS topics"]
  CV["conversions"]
  AM["automsgs"]
  CORE["autonomy core"]

  ROS -->|fromRos| CV --> AM --> CORE
  CORE --> AM -->|toRos| CV --> ROS
```

Bridge 与 `NavigationService` 边界处统一经 `conversions` 进入 core，避免在业务代码中散落字段拷贝。

---

## 扩展新类型

1. 确认 automsgs proto 与 ROS 消息字段可对应
2. 在 `include/autonomy_ros/conversions/<pkg>.hpp` 声明 `fromRos` / `toRos`
3. 在 `src/conversions/<pkg>.cpp` 实现
4. 在 `conversions.hpp` 中 include，并更新 `CMakeLists.txt` / `package.xml` 依赖

---

## 相关文档

- [architecture.md](architecture.md)
- [external_commands.md](external_commands.md)
