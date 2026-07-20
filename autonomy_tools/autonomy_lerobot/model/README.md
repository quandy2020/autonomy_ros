# SAM3 Model
放到这里：`autonomy_lerobot/model/`
- `sam3.pt`
- `bpe_simple_vocab_16e6.txt.gz`
或显式指定权重：
```bash
ros2 launch autonomy_lerobot lerobot_bridge.launch.py mode:=offline sam3_enable:=true sam3_checkpoint:=/absolute/path/to/sam3.pt
```