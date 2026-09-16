# autonomy_ros

ROS 2 桥接：地图 / 路径 / 导航 → RViz & CLI。进程隔离，仅 **autolink**（不链 `libautonomy`）。

```text
autonomy  ←──autolink──→  autonomy_ros  ←──DDS──→  RViz / CLI
```

---

## 📁 布局

```text
/workspace/autonomy/          # 或 ~/workspace/github/autonomy
├── src/
│   ├── autonomy/
│   ├── autonomy_ros/
│   ├── mp3d.scene_dataset_config.json
│   ├── 17DRP5sb8fy/          # *.glb *.navmesh *.house *_semantic.ply
│   └── 8WUmhLawc2A/          # 可选
├── build/  install/  log/
```

**依赖**：ROS Humble · Autolink/automsgs（`/usr/local`）· Habitat-Sim（仿真）

---

## ⬇️ 拉代码

```bash
mkdir -p /workspace/autonomy/src && cd /workspace/autonomy/src
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
git clone --recurse-submodules https://github.com/quandy2020/autonomy_ros.git
```

补子仓：`git submodule update --init --recursive`（在对应仓库内执行）

---

## 🏠 Habitat 场景（MP3D）

1. 申请：[Matterport3D](https://niessner.github.io/Matterport/) → 拿 `download_mp.py`
2. 下载（Python 2.7）：`python download_mp.py --task habitat -o /tmp/mp3d_habitat`
3. 布置：

```bash
WS=/workspace/autonomy/src
cp -a /tmp/mp3d_habitat/17DRP5sb8fy "$WS/"
curl -L -o "$WS/mp3d.scene_dataset_config.json" \
  http://dl.fbaipublicfiles.com/habitat/mp3d/config_v1/mp3d.scene_dataset_config.json
```

4. 配置 `autonomy/autosim/config/default.yaml`：

```yaml
habitat:
  path: /workspace/autonomy/src/17DRP5sb8fy/17DRP5sb8fy.glb
```

---

## 🔨 编译

```bash
source /opt/ros/humble/setup.bash
cd /workspace/autonomy
colcon build --packages-up-to autonomy_ros
source install/setup.bash
```

---

## 🚀 启动

先起 autonomy，再：

```bash
ros2 launch autonomy_ros autonomy.launch.py
# 可选：use_rviz:=false  use_sim_time:=false
```

---

## 🎮 操作

**RViz**：`rviz/autonomy.rviz` → **2D Goal Pose**

```bash
ros2 action send_goal /navigate_to_pose autonomy_ros/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
ros2 service call /cancel_task autonomy_ros/srv/CancelTask "{}"
ros2 topic hz /map /plan
```

---

## 🔌 接口速查

| | 名称 | 用途 |
|--|------|------|
| Action | `navigate_to_pose` / `navigate_through_poses` | 导航 |
| Service | `set_initial_pose` / `cancel_task` | 初值 / 取消 |
| Topic | `goal_pose` `waypoints` `map` `plan` `odom` `cmd_vel` | 目标 / 可视化 |

通道名：`config/parameters.yaml` · 详情：[架构](docs/architecture.md) · [定位](docs/localization.md)
