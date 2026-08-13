# Robot URDF assets

## Go2W (Unitree)

```bash
bash scripts/navrl.sh setup go2w urdf
# 或指定资产源：
GO2W_ASSETS_SRC=/path/to/go2w_description bash scripts/navrl.sh setup go2w urdf
```

输出：`urdf/go2w_description/urdf/go2w_description.urdf`

## Demo

```bash
bash scripts/navrl.sh demo go2w
bash scripts/navrl.sh demo go2w --target-x 2.0 --target-y 0.5 --target-yaw-deg 90
```
