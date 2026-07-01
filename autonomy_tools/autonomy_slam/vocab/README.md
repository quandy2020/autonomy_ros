# ORB 词袋文件

atlas 使用 **FBoW** 词袋，默认文件名：

```
orb_vocab.fbow
```

下载（约 43 MB）：

```bash
./download_vocab.sh
```

或手动下载：

```bash
curl -sL "https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow" \
  -o orb_vocab.fbow
```

### AIST 示例数据集（可选）

用于离线测试 mapping / localization，与 [stella_vslam 教程](https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html) 相同：

```bash
./download_aist_example.sh
# 默认解压到 ../data/aist_living_lab/
# 也可指定目录: ./download_aist_example.sh /path/to/output
```

下载后可用 `run_slam_offline` 回放 rosbag，或直接用 launch 跑 AIST 示例视频：

```bash
# 1) 建图（aist_living_lab_1，输出 map 到 data/aist_living_lab/aist_living_lab_1_map.msg）
ros2 launch autonomy_slam aist_mapping.launch.py

# 2) 定位（aist_living_lab_2，加载上一步的 map）
ros2 launch autonomy_slam aist_localization.launch.py

# 3) 时序定位（temporal mapping）
ros2 launch autonomy_slam aist_localization_temporal.launch.py

# 通用入口（可改 task / sequence 等参数）
ros2 launch autonomy_slam aist_slam.launch.py task:=mapping sequence:=aist_living_lab_1
```

参数与 [stella_vslam 教程](https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html) 对齐：`frame_skip:=3`、全景相机配置 `config/atlas/aist_equirectangular.yaml`。

> **注意**：`orb_vocab.dbow2`（DBoW2 二进制）与 `ORBvoc.txt`（旧版文本格式）均不兼容，请使用上面的 `.fbow` 文件。

启动时也可通过 launch 参数指定路径：

```bash
ros2 launch autonomy_slam slam_mono.launch.py vocab_file:=/path/to/orb_vocab.fbow
```
