# autonomy_lerobot

将在线 ROS 话题或离线 ROS2 bag/mcap 写入 [LeRobot](https://github.com/huggingface/lerobot) v3 数据集。

## 安装

```bash
bash /workspace/autonomy/src/autonomy/docker/install/install_lerobot.sh
pip3 install mcap mcap-ros2-support pyarrow av opencv-python-headless
pip3 install sam3 torch  # 仅在使用 SAM3 时需要

colcon build --packages-select autonomy_lerobot
source install/setup.bash
```

若 `colcon` 报 `--editable`，先执行：

```bash
pip3 install 'setuptools>=61,<80'
```

## 在线录制

```bash
ros2 launch autonomy_lerobot lerobot_bridge.launch.py mode:=online

ros2 service call /lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: true}"
ros2 service call /lerobot_bridge_node/save_episode std_srvs/srv/Trigger {}
ros2 service call /lerobot_bridge_node/set_recording std_srvs/srv/SetBool "{data: false}"
```

在线模式主要配置文件：
- `config/lerobot_bridge.yaml`
- `config/lerobot_collection.yaml`

## 离线转换

默认离线参数在 `config/lerobot_offline.yaml`，按 `input/output/pointcloud/sam3/cleanup` 分块组织，命令行参数会覆盖配置文件。

```bash
ros2 launch autonomy_lerobot lerobot_bridge.launch.py mode:=offline
```

只覆盖单个参数时：

```bash
ros2 launch autonomy_lerobot lerobot_bridge.launch.py \
  mode:=offline \
  input_dir:=/mnt/data4t/rosbag/another_dir
```

也可以直接运行转换脚本：

```bash
ros2 run autonomy_lerobot bag_to_lerobot \
  --input-dir /data/bags \
  --output-root /mnt/data4t/data/lerobot/collection/robot1 \
  --repo-id local/habitat_collection_robot1
```

也支持分阶段流水线：

```bash
ros2 run autonomy_lerobot lerobot_offline_pipeline convert \
  --input-dir /data/bags \
  --output-root /mnt/data4t/data/lerobot/collection/robot1 \
  --repo-id local/habitat_collection_robot1

ros2 run autonomy_lerobot lerobot_offline_pipeline clean \
  --scene-root /mnt/data4t/data/lerobot/collection/robot1/scene_name

ros2 run autonomy_lerobot lerobot_offline_pipeline postprocess \
  --scene-root /mnt/data4t/data/lerobot/collection/robot1/scene_name
```

`convert / clean / postprocess` 现在共用同一套按 `fps + episode_seconds` 的切片规则。
其中阶段式 `convert` 默认只做原始数据抽取；`SAM3` 与点云预览产物交给 `postprocess`。
`postprocess` 默认会为每个 `run` 抽取 `首/中/末` 3 帧，并输出 `baselink` 俯视点云投影。
同时会导出逐帧 `pointcloud/frame_xxxx.ply`；如不需要可加 `--no-ply`。

支持两种输入：
- 直接 `.mcap` 文件
- `rosbag2` 目录树，例如 `rosbag-xxx/rosbag_0.mcap`

常用可选项：
- `--sam3-enable`
- `--save-segmentation-viz`
- `--skip-stationary-runs`

## 输出

输出目录示例：

```text
<output-root>/<scene-name>/
├── meta/info.json
├── meta/report.json
├── run_0/
│   ├── data/chunk-000/file-000.parquet
│   ├── videos/...
│   └── meta/...
└── run_1/
```

数据默认写到 `config/data_paths.yaml` 指定的数据盘目录。

## 可视化

```bash
export HF_HUB_OFFLINE=1
ros2 run autonomy_lerobot lerobot_dataset_viz --robot robot1 --episode-index 0
```

## SAM3 模型

将下面两个文件放到 `autonomy_lerobot/model/`：
- `sam3.pt`
- `bpe_simple_vocab_16e6.txt.gz`
