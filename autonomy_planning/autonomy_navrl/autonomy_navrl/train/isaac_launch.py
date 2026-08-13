"""Isaac Sim AppLauncher helpers for quadruped navigation training."""

from __future__ import annotations

import os
from typing import Any


def resolve_livestream(isaac_cfg: dict[str, Any]) -> int:
    """Map YAML / env to AppLauncher livestream mode.

    0 = off, 1 = WebRTC public, 2 = WebRTC private/local,
    -1 = defer to ``LIVESTREAM`` env (navrl.sh / Docker profile).
    """
    yaml_val: int | None = None
    if 'livestream' in isaac_cfg:
        livestream = isaac_cfg['livestream']
        if isinstance(livestream, bool):
            yaml_val = 2 if livestream else 0
        else:
            yaml_val = int(livestream)
        if yaml_val not in (0, 1, 2):
            raise ValueError(f'isaac.livestream must be 0, 1, or 2 (got {yaml_val})')
        if yaml_val in (1, 2):
            return yaml_val

    env_raw = os.environ.get('LIVESTREAM', '').strip()
    if env_raw.isdigit() and int(env_raw) in (1, 2):
        # Let AppLauncher honor LIVESTREAM when YAML is 0 or unset.
        return -1 if yaml_val in (None, 0) else int(env_raw)

    return int(yaml_val or 0)


def print_webrtc_banner(livestream: int) -> None:
    """Print connection hints for the Isaac Sim WebRTC streaming client."""
    if livestream <= 0:
        return

    public_ip = os.environ.get('PUBLIC_IP', '127.0.0.1')
    port = os.environ.get('LIVESTREAM_PORT', '49100')
    mode = 'public' if livestream == 1 else 'private/local'

    print('[INFO]: Isaac Sim WebRTC livestream enabled.', flush=True)
    print(f'[INFO]:   mode={mode}  server={public_ip}  port={port}', flush=True)
    print(
        '[INFO]:   Connect with NVIDIA Isaac Sim WebRTC Streaming Client '
        '(see scripts/navrl.sh help webrtc).',
        flush=True,
    )
    if livestream == 2:
        print(
            f'[INFO]:   Docker host-network: open client → enter {public_ip} (port {port} if prompted).',
            flush=True,
        )


def bootstrap_isaac_app(config: dict[str, Any]):
    """Launch Isaac Sim before importing torch-dependent modules."""
    from isaaclab.app import AppLauncher

    libcuda_dir = '/usr/lib/x86_64-linux-gnu'
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    if libcuda_dir not in ld_path.split(':'):
        os.environ['LD_LIBRARY_PATH'] = f'{libcuda_dir}:{ld_path}' if ld_path else libcuda_dir

    headless = bool(config.get('headless', True))
    device = _normalize_launcher_device(str(config.get('device', 'cuda:0')))
    isaac_cfg = config.get('isaac', {})
    use_fabric = bool(isaac_cfg.get('use_fabric', device.startswith('cuda')))
    video_cfg = config.get('video', {})
    record_viewport = str(video_cfg.get('source', '')).lower() == 'viewport'
    enable_cameras = bool(isaac_cfg.get('enable_cameras', True))
    if record_viewport and not enable_cameras:
        enable_cameras = True
    livestream = resolve_livestream(isaac_cfg)
    banner_mode = livestream
    if banner_mode <= 0:
        env_raw = os.environ.get('LIVESTREAM', '').strip()
        if env_raw.isdigit():
            banner_mode = int(env_raw)

    launcher_args: dict[str, Any] = {
        'headless': headless,
        'enable_cameras': enable_cameras,
        'device': device,
        'livestream': livestream,
        'video': record_viewport,
    }
    if not use_fabric:
        launcher_args['kit_args'] = (
            '--/physics/fabricUpdateTransformations=false '
            '--/physics/fabricUpdateVelocities=false '
            '--/physics/fabricUpdateJointStates=false '
            '--/physics/fabricUpdateForceSensors=false '
            '--/physics/disableContactProcessing=true'
        )

    print_webrtc_banner(banner_mode)
    app_launcher = AppLauncher(launcher_args)
    if not use_fabric:
        _disable_fabric_runtime()
    return app_launcher.app


def _normalize_launcher_device(device: str) -> str:
    device = device.lower()
    if device.startswith('cuda:'):
        return device
    if device == 'cuda':
        return 'cuda:0'
    return 'cpu'


def _disable_fabric_runtime() -> None:
    """Disable physics Fabric sync on CPU; keep scene delegate for RTX cameras."""
    import carb

    settings = carb.settings.get_settings()
    for key in (
        '/physics/fabricUpdateTransformations',
        '/physics/fabricUpdateVelocities',
        '/physics/fabricUpdateJointStates',
        '/physics/fabricUpdateForceSensors',
    ):
        settings.set_bool(key, False)
