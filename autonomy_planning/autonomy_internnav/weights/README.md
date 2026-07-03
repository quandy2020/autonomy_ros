# NavDP Weights

Place NavDP checkpoint files in this directory.

## Default model

| File | Description |
|------|-------------|
| `navdp-cross-modal.ckpt` | NavDP cross-modal navigation policy (default) |

Download from the [NavDP model form](https://docs.google.com/forms/d/e/1FAIpQLSdl3RvajO5AohwWZL5C0yM-gkSqrNaLGp1OzN9oF24oNLfikw/viewform?usp=dialog) and save as `navdp-cross-modal.ckpt`.

## Usage

Place the checkpoint in the source tree (not installed by colcon):

`src/autonomy_ros/autonomy_planning/autonomy_internnav/weights/navdp-cross-modal.ckpt`

Or set an environment variable:

```bash
export NAVDP_CHECKPOINT=/path/to/navdp-cross-modal.ckpt
# or
export AUTONOMY_INTERNNAV_WEIGHTS=/path/to/weights_dir
```

Launch with the default checkpoint name:

```bash
ros2 launch autonomy_internnav internnav.launch.py
```

Or override:

```bash
ros2 launch autonomy_internnav internnav.launch.py \
  checkpoint:=/path/to/your_navdp.ckpt
```

If a previous build left a broken install symlink, clean and rebuild:

```bash
rm -rf build/autonomy_internnav install/autonomy_internnav
colcon build --packages-select autonomy_internnav --symlink-install --allow-overriding autonomy_internnav
```

## Training

NavDP training code is integrated from InternNav under `autonomy_internnav/train/`.
Training checkpoints are saved as `navdp.ckpt` in `checkpoints/<run_name>/ckpts/`.

Install training dependencies:

```bash
bash scripts/install_train_deps.sh
```

Launch training (single GPU):

```bash
bash scripts/train/start_train.sh --name my_navdp_run --gpus 1
```

Or after colcon install:

```bash
train_navdp --name my_navdp_run
```

Edit default hyperparameters in `autonomy_internnav/train/configs/navdp_default.py`
(dataset paths, batch size, learning rate, etc.).
