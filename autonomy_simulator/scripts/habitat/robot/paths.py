"""Robot URDF/Xacro asset discovery for dynamic Habitat actors."""

from __future__ import annotations

import math
import os
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from habitat.config import Config
from habitat.robot.quadruped_controller import SPOT_STAND_POSE, supports_quadruped_gait


ROBOT_TYPE_METADATA: dict[str, dict[str, float | int | str]] = {
    'spot': {
        'radius': 0.35,
        'height': 0.84,
        'semantic_id': 260,
    },
    'turtlebot3': {
        'radius': 0.18,
        'height': 0.32,
        'semantic_id': 251,
    },
    'jackal': {
        'radius': 0.28,
        'height': 0.23,
        'semantic_id': 252,
    },
    'husky': {
        'radius': 0.42,
        'height': 0.39,
        'semantic_id': 253,
    },
    'stretch': {
        'radius': 0.28,
        'height': 1.10,
        'semantic_id': 254,
    },
}


ROBOT_JOINT_POSE_PRESETS: dict[str, dict[str, float]] = {
    'spot': dict(SPOT_STAND_POSE),
}


def _parse_override_map(raw: str, *, cast):
    result: dict[str, float | int] = {}
    for part in str(raw).split(','):
        item = part.strip()
        if not item or '=' not in item:
            continue
        key, value = item.split('=', 1)
        key = key.strip()
        value = value.strip()
        if not key or not value:
            continue
        try:
            result[key] = cast(value)
        except Exception:
            continue
    return result


def _parse_xyz(raw: str | None) -> tuple[float, float, float]:
    parts = (raw or '0 0 0').split()
    vals = [float(part) for part in parts[:3]]
    while len(vals) < 3:
        vals.append(0.0)
    return (vals[0], vals[1], vals[2])


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)]
        for row in range(4)
    ]


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _axis_angle_matrix(axis: tuple[float, float, float], angle: float) -> list[list[float]]:
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-9 or abs(angle) < 1e-9:
        return _rpy_matrix(0.0, 0.0, 0.0)
    ax /= norm
    ay /= norm
    az /= norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _transform_matrix(
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> list[list[float]]:
    mat = _rpy_matrix(*rpy)
    mat[0][3] = xyz[0]
    mat[1][3] = xyz[1]
    mat[2][3] = xyz[2]
    return mat


def _matrix_to_xyz_rpy(mat: list[list[float]]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    x = float(mat[0][3])
    y = float(mat[1][3])
    z = float(mat[2][3])
    sy = -mat[2][0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    singular = cy < 1e-6
    if not singular:
        roll = math.atan2(mat[2][1], mat[2][2])
        pitch = math.atan2(sy, cy)
        yaw = math.atan2(mat[1][0], mat[0][0])
    else:
        roll = math.atan2(-mat[1][2], mat[1][1])
        pitch = math.atan2(sy, cy)
        yaw = 0.0
    return (x, y, z), (roll, pitch, yaw)


def _temp_asset_dir() -> Path:
    path = Path(tempfile.gettempdir()) / 'autonomy_simulator_robot_assets'
    path.mkdir(parents=True, exist_ok=True)
    return path


def _official_robot_source_candidates() -> dict[str, list[Path]]:
    return {
        'spot': [
            Path('/mnt/data4t/Datasets/humanoids/robots/hab_spot_arm/urdf/hab_spot_arm.urdf'),
            Path('/workspace/autonomy/data/robots/hab_spot_arm/urdf/hab_spot_arm.urdf'),
        ],
        'fetch': [
            Path('/mnt/data4t/Datasets/humanoids/robots/hab_fetch/robots/hab_fetch.urdf'),
            Path('/mnt/data4t/Datasets/humanoids/robots/hab_fetch/urdf/hab_fetch.urdf'),
        ],
        'stretch': [
            Path('/mnt/data4t/Datasets/humanoids/robots/hab_stretch/urdf/hab_stretch.urdf'),
        ],
    }


def _official_robot_sources() -> dict[str, Path]:
    resolved: dict[str, Path] = {}
    for name, candidates in _official_robot_source_candidates().items():
        for candidate in candidates:
            if candidate.is_file():
                resolved[name] = candidate
                break
    return resolved


def official_robot_asset_names() -> list[str]:
    return list(_official_robot_sources().keys())


def robot_joint_pose_preset(asset_name: str) -> dict[str, float]:
    return dict(ROBOT_JOINT_POSE_PRESETS.get(_canonical_robot_name(asset_name), {}))


def robot_uses_full_articulation(asset_name: str) -> bool:
    return supports_quadruped_gait(_canonical_robot_name(asset_name))


def _canonical_robot_name(asset_name: str) -> str:
    raw = str(asset_name).strip().lower()
    aliases = {
        'hab_spot_arm': 'spot',
        'hab_fetch': 'fetch',
        'hab_stretch': 'stretch',
    }
    return aliases.get(raw, raw)


def _is_binary_stl(data: bytes) -> bool:
    if len(data) < 84:
        return False
    tri_count = struct.unpack('<I', data[80:84])[0]
    return len(data) == 84 + tri_count * 50


def _stl_to_obj(stl_path: Path, obj_path: Path) -> Path:
    data = stl_path.read_bytes()
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    if _is_binary_stl(data):
        tri_count = struct.unpack('<I', data[80:84])[0]
        offset = 84
        for _ in range(tri_count):
            offset += 12  # normal
            tri: list[tuple[float, float, float]] = []
            for _ in range(3):
                vx, vy, vz = struct.unpack('<fff', data[offset:offset + 12])
                tri.append((vx, vy, vz))
                vertices.append((vx, vy, vz))
                offset += 12
            base = len(vertices) - 2
            faces.append((base, base + 1, base + 2))
            offset += 2  # attribute byte count
    else:
        tri: list[tuple[float, float, float]] = []
        for line in data.decode('utf-8', errors='ignore').splitlines():
            stripped = line.strip()
            if not stripped.startswith('vertex '):
                continue
            parts = stripped.split()
            if len(parts) != 4:
                continue
            vx, vy, vz = (float(parts[1]), float(parts[2]), float(parts[3]))
            tri.append((vx, vy, vz))
            vertices.append((vx, vy, vz))
            if len(tri) == 3:
                base = len(vertices) - 2
                faces.append((base, base + 1, base + 2))
                tri = []
    obj_path.parent.mkdir(parents=True, exist_ok=True)
    with obj_path.open('w', encoding='utf-8') as f:
        f.write(f'# Converted from {stl_path.name}\n')
        for vx, vy, vz in vertices:
            f.write(f'v {vx:.9f} {vy:.9f} {vz:.9f}\n')
        for a, b, c in faces:
            f.write(f'f {a} {b} {c}\n')
    return obj_path


def _convert_mesh_assets(text: str, asset_name: str) -> str:
    root = ET.fromstring(text)
    out_dir = _temp_asset_dir() / f'{asset_name}_mesh_cache'
    changed = False
    for mesh in root.findall('.//mesh'):
        filename = str(mesh.attrib.get('filename', '')).strip()
        if not filename:
            continue
        source = Path(filename)
        if source.suffix.lower() != '.stl' or not source.is_file():
            continue
        target = out_dir / f'{source.stem}.obj'
        if not target.exists():
            _stl_to_obj(source, target)
        mesh.set('filename', target.as_posix())
        changed = True
    if not changed:
        return text
    return ET.tostring(root, encoding='unicode')


def _converted_mesh_path(source: Path, asset_name: str) -> Path:
    out_dir = _temp_asset_dir() / f'{asset_name}_mesh_cache'
    target = out_dir / f'{source.stem}.obj'
    if source.suffix.lower() == '.stl' and source.is_file() and not target.exists():
        _stl_to_obj(source, target)
    return target if target.exists() else source


def _rewrite_relative_mesh_paths(text: str, asset_source: Path) -> str:
    root = ET.fromstring(text)
    base_dir = asset_source.parent
    changed = False
    for mesh in root.findall('.//mesh'):
        filename = str(mesh.attrib.get('filename', '')).strip()
        if not filename or '://' in filename:
            continue
        candidate = (base_dir / filename).resolve()
        if candidate.exists():
            mesh.set('filename', candidate.as_posix())
            changed = True
    if not changed:
        return text
    return ET.tostring(root, encoding='unicode')


def _marker_spec(asset_name: str, asset_root: Path) -> dict[str, object] | None:
    pkg_root = asset_root
    if asset_name == 'turtlebot3':
        return {
            'mesh': str((_package_root() / 'models' / 'turtlebot3_model' / 'meshes' / 'waffle_base.dae').resolve()),
            'scale': (0.001, 0.001, 0.001),
            'offset_xyz': (-0.064, 0.0, 0.0),
            'offset_rpy': (0.0, 0.0, 0.0),
        }
    if asset_name == 'spot':
        mesh = (pkg_root.parent / 'meshesColored' / 'base.glb').resolve()
        return {
            'mesh': str(mesh),
            'scale': (1.0, 1.0, 1.0),
            'offset_xyz': (0.0, 0.0, 0.0),
            'offset_rpy': (0.0, 0.0, 0.0),
        }
    if asset_name == 'jackal':
        return {
            'parts': [
                {
                    'type': 'box',
                    'size': (0.420, 0.310, 0.184),
                    'offset_xyz': (0.0, 0.0, 0.092),
                    'offset_rpy': (0.0, 0.0, 0.0),
                    'color': (0.78, 0.55, 0.52, 0.95),
                },
            ],
        }
    if asset_name == 'husky':
        return {
            'mesh': str((pkg_root / 'meshes' / 'base_link.dae').resolve()),
            'scale': (1.0, 1.0, 1.0),
            'offset_xyz': (0.0, 0.0, 0.0),
            'offset_rpy': (0.0, 0.0, 0.0),
        }
    if asset_name == 'stretch':
        mesh = pkg_root / 'batch' / 'nina' / 'meshes' / 'base_link.STL'
        if not mesh.is_file():
            candidates = sorted(pkg_root.rglob('base_link.STL'))
            mesh = candidates[0] if candidates else mesh
        return {
            'mesh': str(mesh.resolve()),
            'scale': (1.0, 1.0, 1.0),
            'offset_xyz': (0.0, 0.0, 0.0),
            'offset_rpy': (0.0, 0.0, 0.0),
        }
    return None


def _package_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _autonomy_ros_root() -> Path:
    return _package_root().parent


def _organized_robot_root() -> Path:
    return Path('/mnt/data4t/Datasets/Robots/organized')


def resolve_robot_asset_root(cfg: Config) -> str:
    if str(cfg.robot_asset_root).strip():
        return str(cfg.robot_asset_root).rstrip('/')
    if _organized_robot_root().is_dir():
        return str(_organized_robot_root())
    return str(_package_root() / 'urdf')


def robot_asset_urdf(asset_root: str, asset_name: str) -> str:
    return os.path.join(asset_root, f'{asset_name}.urdf')


def is_robot_asset_available(asset_root: str, asset_name: str) -> bool:
    asset_name = _canonical_robot_name(asset_name)
    if asset_name in _official_robot_sources():
        return True
    root = Path(asset_root)
    return os.path.isfile(robot_asset_urdf(asset_root, asset_name)) or (
        (root / asset_name / 'description').exists()
    )


def list_available_robot_assets(asset_root: str) -> list[str]:
    root = Path(asset_root)
    official = list(_official_robot_sources().keys())
    if not root.is_dir():
        return official
    organized = [
        entry.name
        for entry in sorted(root.iterdir())
        if entry.is_dir() and (entry / 'description').exists()
    ]
    organized = [name for name in organized if _canonical_robot_name(name) not in official]
    if official or organized:
        return official + organized
    blocked = {'habitat'}
    direct = sorted(
        entry.stem
        for entry in root.glob('*.urdf')
        if entry.is_file() and entry.stem not in blocked
    )
    direct = [name for name in direct if _canonical_robot_name(name) not in official]
    return official + direct


def resolve_robot_asset_roster(cfg: Config) -> list[str]:
    root = resolve_robot_asset_root(cfg)
    requested = [
        _canonical_robot_name(part.strip())
        for part in str(cfg.robot_asset_types).split(',')
        if part.strip()
    ]
    if requested:
        roster = [name for name in requested if is_robot_asset_available(root, name)]
        if roster:
            return roster
    discovered = list_available_robot_assets(root)
    if discovered:
        return discovered
    fallback = str(cfg.robot_asset_type).strip()
    if fallback and is_robot_asset_available(root, fallback):
        return [fallback]
    return []


def robot_assets_available(cfg: Config) -> tuple[bool, str]:
    root = resolve_robot_asset_root(cfg)
    roster = resolve_robot_asset_roster(cfg)
    if not roster:
        return False, f'no valid robot assets under {root}'
    return True, root


def _resolve_asset_source(asset_root: str, asset_name: str) -> Path:
    asset_name = _canonical_robot_name(asset_name)
    official = _official_robot_sources()
    if asset_name in official:
        return official[asset_name]
    root = Path(asset_root)
    direct_urdf = root / f'{asset_name}.urdf'
    if direct_urdf.is_file():
        return direct_urdf

    organized_dir = root / asset_name
    description = organized_dir / 'description'
    if description.exists():
        preferred_stems = {
            'turtlebot3': 'turtlebot3_waffle',
            'jackal': 'jackal',
            'husky': 'husky',
            'stretch': 'stretch_base_imu',
        }
        preferred = preferred_stems.get(asset_name, asset_name)
        preferred_urdf = description / 'urdf' / f'{preferred}.urdf'
        if preferred_urdf.is_file():
            return preferred_urdf
        preferred_xacro = description / 'urdf' / f'{preferred}.xacro'
        if preferred_xacro.is_file():
            return preferred_xacro
        preferred_urdf_xacro = description / 'urdf' / f'{preferred}.urdf.xacro'
        if preferred_urdf_xacro.is_file():
            return preferred_urdf_xacro
        direct_pkg_urdf = description / 'urdf' / f'{asset_name}.urdf'
        if direct_pkg_urdf.is_file():
            return direct_pkg_urdf
        direct_pkg_xacro = description / 'urdf' / f'{asset_name}.urdf.xacro'
        if direct_pkg_xacro.is_file():
            return direct_pkg_xacro
        candidates = sorted(
            path for path in description.rglob('*.urdf')
            if 'accessories' not in path.parts
        )
        if candidates:
            return candidates[0]
        xacro_candidates = sorted(
            path for path in description.rglob('*.xacro')
            if 'accessories' not in path.parts
        )
        if xacro_candidates:
            return xacro_candidates[0]
    raise FileNotFoundError(f'robot asset source not found for {asset_name} under {asset_root}')


def _rewrite_package_uris(text: str, asset_source: Path) -> str:
    # Legacy turtlebot URDFs use package://autonomy_ros/models/... even though the
    # assets live under autonomy_simulator/models/.
    text = text.replace(
        'package://autonomy_ros/models/',
        f'{(_package_root() / "models").as_posix()}/',
    )
    text = text.replace('package://autonomy_ros/', f'{_autonomy_ros_root().as_posix()}/')
    text = text.replace(
        'package://autonomy_simulator/',
        f'{_package_root().as_posix()}/',
    )
    pkg_root = asset_source.parent.parent if asset_source.parent.name == 'urdf' else asset_source.parent
    if pkg_root.is_dir():
        for package_name in (
            'jackal_description',
            'husky_description',
            'stretch_description',
            'turtlebot3_description',
        ):
            text = text.replace(
                f'package://{package_name}/',
                f'{pkg_root.as_posix()}/',
            )
    return _rewrite_relative_mesh_paths(text, asset_source)


def _materialize_xacro_to_urdf(xacro_path: Path, output_path: Path) -> None:
    for command in (
        ['python3', '-m', 'xacro', str(xacro_path)],
        ['ros2', 'run', 'xacro', 'xacro', str(xacro_path)],
    ):
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode == 0:
            output_path.write_text(result.stdout, encoding='utf-8')
            return
    stderr = (result.stderr or '').strip()
    raise RuntimeError(
        'failed to expand xacro for dynamic robot asset '
        f'{xacro_path}: {stderr or "xacro is not installed in the runtime environment"}'
    )


def _jackal_visual_urdf(pkg_root: Path) -> str | None:
    wheel_visuals = ''.join(
        (
            f'<visual><origin xyz="{x} {y} 0.098" rpy="1.570796 0 0"/>'
            f'<geometry><cylinder radius="0.098" length="0.045"/></geometry>'
            f'<material name="jackal_wheel"/></visual>'
        )
        for x, y in (
            (0.180, 0.165),
            (0.180, -0.165),
            (-0.180, 0.165),
            (-0.180, -0.165),
        )
    )
    return (
        f'<robot name="jackal">'
        f'<material name="jackal_body"><color rgba="0.78 0.55 0.52 0.95"/></material>'
        f'<material name="jackal_wheel"><color rgba="0.12 0.12 0.12 1.0"/></material>'
        f'<link name="base_link">'
        f'<visual><origin xyz="0 0 0.092" rpy="0 0 0"/>'
        f'<geometry><box size="0.420 0.310 0.184"/></geometry>'
        f'<material name="jackal_body"/></visual>'
        f'{wheel_visuals}'
        f'<collision><origin xyz="0 0 0.092" rpy="0 0 0"/>'
        f'<geometry><box size="0.420 0.310 0.184"/></geometry></collision>'
        f'<inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="16.523"/>'
        f'<inertia ixx="0.3136" ixy="0" ixz="0" iyy="0.3922" iyz="0" izz="0.4485"/>'
        f'</inertial></link></robot>'
    )


def _husky_visual_urdf(pkg_root: Path) -> str | None:
    base_mesh = pkg_root / 'meshes' / 'base_link.dae'
    wheel_visuals = ''.join(
        (
            f'<visual><origin xyz="{x} {y} 0.165" rpy="1.570796 0 0"/>'
            f'<geometry><cylinder radius="0.165" length="0.065"/></geometry>'
            f'<material name="husky_wheel"/></visual>'
        )
        for x, y in (
            (0.285, 0.285),
            (0.285, -0.285),
            (-0.285, 0.285),
            (-0.285, -0.285),
        )
    )
    base_visual = (
        f'<visual><origin xyz="0 0 0.140" rpy="0 0 0"/>'
        f'<geometry><box size="0.99 0.57 0.20"/></geometry>'
        f'<material name="husky_body"/></visual>'
    )
    if base_mesh.is_file():
        base_visual = (
            f'<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
            f'<geometry><mesh filename="{base_mesh.as_posix()}"/></geometry>'
            f'</visual>'
        )
    return (
        f'<robot name="husky">'
        f'<material name="husky_body"><color rgba="0.82 0.82 0.84 1.0"/></material>'
        f'<material name="husky_wheel"><color rgba="0.12 0.12 0.12 1.0"/></material>'
        f'<link name="base_link">'
        f'{base_visual}'
        f'{wheel_visuals}'
        f'<collision><origin xyz="0 0 0.140" rpy="0 0 0"/>'
        f'<geometry><box size="0.9874 0.5709 0.28"/></geometry></collision>'
        f'<inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="46.034"/>'
        f'<inertia ixx="0.6022" ixy="0" ixz="0" iyy="1.7386" iyz="0" izz="2.0296"/>'
        f'</inertial></link></robot>'
    )


def _spot_robot_model_urdf() -> str:
    links = {
        'base': (
            '<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
            '<geometry><box size="0.58 0.18 0.16"/></geometry>'
            '<material name="spot_body"/></visual>'
        ),
        'fl.hip': (
            '<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
            '<geometry><box size="0.08 0.05 0.05"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'fr.hip': (
            '<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
            '<geometry><box size="0.08 0.05 0.05"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'hl.hip': (
            '<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
            '<geometry><box size="0.08 0.05 0.05"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'hr.hip': (
            '<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
            '<geometry><box size="0.08 0.05 0.05"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'fl.uleg': (
            '<visual><origin xyz="0 0 -0.16" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.035" length="0.32"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'fr.uleg': (
            '<visual><origin xyz="0 0 -0.16" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.035" length="0.32"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'hl.uleg': (
            '<visual><origin xyz="0 0 -0.16" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.035" length="0.32"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'hr.uleg': (
            '<visual><origin xyz="0 0 -0.16" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.035" length="0.32"/></geometry>'
            '<material name="spot_leg"/></visual>'
        ),
        'fl.lleg': (
            '<visual><origin xyz="0 0 -0.17" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.028" length="0.34"/></geometry>'
            '<material name="spot_shin"/></visual>'
        ),
        'fr.lleg': (
            '<visual><origin xyz="0 0 -0.17" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.028" length="0.34"/></geometry>'
            '<material name="spot_shin"/></visual>'
        ),
        'hl.lleg': (
            '<visual><origin xyz="0 0 -0.17" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.028" length="0.34"/></geometry>'
            '<material name="spot_shin"/></visual>'
        ),
        'hr.lleg': (
            '<visual><origin xyz="0 0 -0.17" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.028" length="0.34"/></geometry>'
            '<material name="spot_shin"/></visual>'
        ),
        'arm0.link_sh0': (
            '<visual><origin xyz="0.10 0 0" rpy="0 1.570796 0"/>'
            '<geometry><cylinder radius="0.028" length="0.20"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
        'arm0.link_sh1': (
            '<visual><origin xyz="0.09 0 0" rpy="0 1.570796 0"/>'
            '<geometry><cylinder radius="0.026" length="0.18"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
        'arm0.link_hr0': (
            '<visual><origin xyz="0.07 0 0" rpy="0 1.570796 0"/>'
            '<geometry><cylinder radius="0.024" length="0.14"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
        'arm0.link_el0': (
            '<visual><origin xyz="0.17 0 0" rpy="0 1.570796 0"/>'
            '<geometry><cylinder radius="0.022" length="0.34"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
        'arm0.link_el1': (
            '<visual><origin xyz="0.20 0 0.04" rpy="0 1.570796 0"/>'
            '<geometry><cylinder radius="0.020" length="0.42"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
        'arm0.link_wr0': (
            '<visual><origin xyz="0.05 0 0" rpy="0 1.570796 0"/>'
            '<geometry><cylinder radius="0.018" length="0.10"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
        'arm0.link_wr1': (
            '<visual><origin xyz="0.04 0 0" rpy="0 1.570796 0"/>'
            '<geometry><cylinder radius="0.018" length="0.08"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
        'arm0.link_fngr': (
            '<visual><origin xyz="0.04 0 0" rpy="0 0 0"/>'
            '<geometry><box size="0.08 0.03 0.03"/></geometry>'
            '<material name="spot_arm"/></visual>'
        ),
    }
    joints = [
        ('fl.hx', 'revolute', 'base', 'fl.hip', '0.29785 0.05500 0.00000', '0 0 0', '1 0 0'),
        ('fl.hy', 'revolute', 'fl.hip', 'fl.uleg', '0.0 0.110945 0.0', '0 0 0', '0 1 0'),
        ('fl.kn', 'revolute', 'fl.uleg', 'fl.lleg', '0.025 0.000 -0.3205', '0 0 0', '0 1 0'),
        ('fr.hx', 'revolute', 'base', 'fr.hip', '0.29785 -0.05500 0.00000', '0 0 0', '1 0 0'),
        ('fr.hy', 'revolute', 'fr.hip', 'fr.uleg', '0.0 -0.110945 0.0', '0 0 0', '0 1 0'),
        ('fr.kn', 'revolute', 'fr.uleg', 'fr.lleg', '0.025 0.000 -0.3205', '0 0 0', '0 1 0'),
        ('hl.hx', 'revolute', 'base', 'hl.hip', '-0.29785 0.05500 0.00000', '0 0 0', '1 0 0'),
        ('hl.hy', 'revolute', 'hl.hip', 'hl.uleg', '0.0 0.110945 0.0', '0 0 0', '0 1 0'),
        ('hl.kn', 'revolute', 'hl.uleg', 'hl.lleg', '0.025 0.000 -0.3205', '0 0 0', '0 1 0'),
        ('hr.hx', 'revolute', 'base', 'hr.hip', '-0.29785 -0.05500 0.00000', '0 0 0', '1 0 0'),
        ('hr.hy', 'revolute', 'hr.hip', 'hr.uleg', '0.0 -0.110945 0.0', '0 0 0', '0 1 0'),
        ('hr.kn', 'revolute', 'hr.uleg', 'hr.lleg', '0.025 0.000 -0.3205', '0 0 0', '0 1 0'),
        ('arm0.sh0', 'revolute', 'base', 'arm0.link_sh0', '0.292 0.0 0.188', '0 0 0', '0 0 1'),
        ('arm0.sh1', 'revolute', 'arm0.link_sh0', 'arm0.link_sh1', '0.0 0.0 0.0', '0 0 0', '0 1 0'),
        ('arm0.hr0', 'revolute', 'arm0.link_sh1', 'arm0.link_hr0', '0.0 0.0 0.0', '0 0 0', '1 0 0'),
        ('arm0.el0', 'revolute', 'arm0.link_hr0', 'arm0.link_el0', '0.3385 0 0', '0 0 0', '0 1 0'),
        ('arm0.el1', 'revolute', 'arm0.link_el0', 'arm0.link_el1', '0.40330 0.0 0.0750', '0 0 0', '1 0 0'),
        ('arm0.wr0', 'revolute', 'arm0.link_el1', 'arm0.link_wr0', '0.0 0.0 0.0', '0 0 0', '0 1 0'),
        ('arm0.wr1', 'revolute', 'arm0.link_wr0', 'arm0.link_wr1', '0.0 0.0 0.0', '0 0 0', '1 0 0'),
        ('arm0.f1x', 'revolute', 'arm0.link_wr1', 'arm0.link_fngr', '0.11745 0 0.014820', '0 0 0', '0 1 0'),
    ]
    link_xml = ''.join(
        f'<link name="{name}">{visual}</link>'
        for name, visual in links.items()
    )
    joint_xml = ''.join(
        f'<joint name="{name}" type="{joint_type}">'
        f'<origin xyz="{xyz}" rpy="{rpy}"/>'
        f'<axis xyz="{axis}"/>'
        f'<parent link="{parent}"/>'
        f'<child link="{child}"/>'
        f'<limit lower="-3.141593" upper="3.141593" effort="1000.0" velocity="1000.0"/>'
        f'</joint>'
        for name, joint_type, parent, child, xyz, rpy, axis in joints
    )
    return (
        '<robot name="spot">'
        '<material name="spot_body"><color rgba="0.22 0.22 0.24 1.0"/></material>'
        '<material name="spot_leg"><color rgba="0.88 0.72 0.34 1.0"/></material>'
        '<material name="spot_shin"><color rgba="0.18 0.18 0.18 1.0"/></material>'
        '<material name="spot_arm"><color rgba="0.72 0.72 0.76 1.0"/></material>'
        f'{link_xml}{joint_xml}'
        '</robot>'
    )


def _simple_vendor_robot_urdf(asset_name: str, asset_source: Path) -> str | None:
    """Return a minimal single-link URDF for known vendor robots.

    This avoids complex xacro/package dependency chains when we only need a
    kinematic obstacle mesh inside Habitat.
    """
    pkg_root = asset_source.parent.parent if asset_source.parent.name == 'urdf' else asset_source.parent
    if asset_name == 'jackal':
        return _jackal_visual_urdf(pkg_root)
    if asset_name == 'husky':
        return _husky_visual_urdf(pkg_root)
    if asset_name == 'stretch':
        mesh = pkg_root / 'batch' / 'nina' / 'meshes' / 'base_link.STL'
        if not mesh.is_file():
            candidates = sorted(pkg_root.rglob('base_link.STL'))
            mesh = candidates[0] if candidates else mesh
        if mesh.is_file():
            return (
                f'<robot name="stretch">'
                f'<link name="base_link">'
                f'<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
                f'<geometry><mesh filename="{mesh.as_posix()}"/></geometry></visual>'
                f'<collision><origin xyz="0 0 0.20" rpy="0 0 0"/>'
                f'<geometry><box size="0.36 0.34 0.40"/></geometry></collision>'
                f'<inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="24.0"/>'
                f'<inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>'
                f'</inertial></link></robot>'
            )
    return None


def _single_link_urdf(text: str, asset_name: str) -> str:
    """Flatten a robot URDF to one base link for Habitat compatibility.

    Habitat's URDF parser is stricter than RViz/Gazebo and often rejects vendor
    robot descriptions with empty helper links or complex joint chains. For
    dynamic obstacles we only need a visual/collision hull, so we merge all
    link visuals/collisions into one kinematic base link.
    """
    root = ET.fromstring(text)
    out_root = ET.Element('robot', {'name': root.attrib.get('name', asset_name)})
    pose_preset = ROBOT_JOINT_POSE_PRESETS.get(asset_name, {})

    material_names: set[str] = set()
    for material in root.findall('material'):
        name = material.attrib.get('name', '').strip()
        if name and name not in material_names:
            out_root.append(material)
            material_names.add(name)

    base_link = ET.SubElement(out_root, 'link', {'name': 'base_link'})
    inertial = None
    visual_count = 0
    collision_count = 0
    link_map = {str(link.attrib.get('name', '')).strip(): link for link in root.findall('link')}
    child_to_joint: dict[str, ET.Element] = {}
    for joint in root.findall('joint'):
        child = joint.find('child')
        child_name = str(child.attrib.get('link', '')).strip() if child is not None else ''
        if child_name:
            child_to_joint[child_name] = joint

    link_world: dict[str, list[list[float]]] = {}

    def world_transform(link_name: str) -> list[list[float]]:
        if link_name in link_world:
            return link_world[link_name]
        joint = child_to_joint.get(link_name)
        if joint is None:
            link_world[link_name] = _transform_matrix()
            return link_world[link_name]
        parent = joint.find('parent')
        parent_name = str(parent.attrib.get('link', '')).strip() if parent is not None else ''
        origin = joint.find('origin')
        xyz = _parse_xyz(origin.attrib.get('xyz') if origin is not None else None)
        rpy = _parse_xyz(origin.attrib.get('rpy') if origin is not None else None)
        local = _transform_matrix(xyz, rpy)
        joint_type = str(joint.attrib.get('type', '')).strip().lower()
        if joint_type in {'revolute', 'continuous', 'prismatic'}:
            angle = float(pose_preset.get(str(joint.attrib.get('name', '')).strip(), 0.0))
            axis_elem = joint.find('axis')
            axis = _parse_xyz(axis_elem.attrib.get('xyz') if axis_elem is not None else '0 0 1')
            if joint_type == 'prismatic':
                local = _mat_mul(local, _transform_matrix(
                    (axis[0] * angle, axis[1] * angle, axis[2] * angle),
                    (0.0, 0.0, 0.0),
                ))
            else:
                local = _mat_mul(local, _axis_angle_matrix(axis, angle))
        parent_tf = world_transform(parent_name) if parent_name else _transform_matrix()
        link_world[link_name] = _mat_mul(parent_tf, local)
        return link_world[link_name]

    for link_name, link in link_map.items():
        if inertial is None:
            candidate_inertial = link.find('inertial')
            if candidate_inertial is not None:
                inertial = candidate_inertial
        link_tf = world_transform(link_name)
        for tag_name in ('visual', 'collision'):
            for elem in link.findall(tag_name):
                merged = ET.fromstring(ET.tostring(elem, encoding='unicode'))
                origin = merged.find('origin')
                local_xyz = _parse_xyz(origin.attrib.get('xyz') if origin is not None else None)
                local_rpy = _parse_xyz(origin.attrib.get('rpy') if origin is not None else None)
                global_tf = _mat_mul(link_tf, _transform_matrix(local_xyz, local_rpy))
                xyz, rpy = _matrix_to_xyz_rpy(global_tf)
                if origin is None:
                    origin = ET.Element('origin')
                    merged.insert(0, origin)
                origin.set('xyz', f'{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}')
                origin.set('rpy', f'{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}')
                base_link.append(merged)
                if tag_name == 'visual':
                    visual_count += 1
                else:
                    collision_count += 1

    if inertial is None:
        inertial = ET.fromstring(
            '<inertial>'
            '<origin xyz="0 0 0" rpy="0 0 0"/>'
            '<mass value="1.0"/>'
            '<inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>'
            '</inertial>'
        )
    base_link.append(inertial)

    if visual_count == 0 and collision_count == 0:
        base_link.append(
            ET.fromstring(
                '<collision><origin xyz="0 0 0" rpy="0 0 0"/>'
                '<geometry><box size="0.5 0.5 0.5"/></geometry></collision>'
            )
        )
        base_link.append(
            ET.fromstring(
                '<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
                '<geometry><box size="0.5 0.5 0.5"/></geometry>'
                '<material name="fallback_gray"><color rgba="0.5 0.5 0.5 1.0"/></material>'
                '</visual>'
            )
        )

    return ET.tostring(out_root, encoding='unicode')


def materialize_robot_urdf(cfg: Config, asset_name: str) -> str:
    """Write a temporary URDF with package URIs resolved to local paths."""
    asset_source = _resolve_asset_source(resolve_robot_asset_root(cfg), asset_name)
    temp_dir = _temp_asset_dir()
    out = temp_dir / f'{asset_name}.resolved.urdf'
    simple_urdf = _simple_vendor_robot_urdf(asset_name, asset_source)
    if simple_urdf is not None:
        simple_urdf = _convert_mesh_assets(simple_urdf, asset_name)
        out.write_text(simple_urdf, encoding='utf-8')
        return str(out)
    if asset_source.suffix == '.xacro':
        _materialize_xacro_to_urdf(asset_source, out)
        text = out.read_text(encoding='utf-8')
    else:
        text = asset_source.read_text(encoding='utf-8')
    text = _rewrite_package_uris(text, asset_source)
    text = _convert_mesh_assets(text, asset_name)
    if not robot_uses_full_articulation(asset_name):
        text = _single_link_urdf(text, asset_name)
    out.write_text(text, encoding='utf-8')
    return str(out)


def _prefix_robot_names(text: str, prefix: str) -> str:
    if not prefix:
        return text
    root = ET.fromstring(text)
    for link in root.findall('link'):
        name = str(link.attrib.get('name', '')).strip()
        if name:
            link.set('name', f'{prefix}{name}')
    for joint in root.findall('joint'):
        name = str(joint.attrib.get('name', '')).strip()
        if name:
            joint.set('name', f'{prefix}{name}')
        parent = joint.find('parent')
        if parent is not None and 'link' in parent.attrib:
            parent.set('link', f'{prefix}{parent.attrib["link"]}')
        child = joint.find('child')
        if child is not None and 'link' in child.attrib:
            child.set('link', f'{prefix}{child.attrib["link"]}')
    root.set('name', f'{prefix}{root.attrib.get("name", "robot")}')
    return ET.tostring(root, encoding='unicode')


def materialize_robot_model_urdf(cfg: Config, asset_name: str, *, prefix: str = '') -> str:
    asset_source = _resolve_asset_source(resolve_robot_asset_root(cfg), asset_name)
    model_urdf = _spot_robot_model_urdf() if _canonical_robot_name(asset_name) == 'spot' else None
    simple_urdf = _simple_vendor_robot_urdf(asset_name, asset_source)
    if model_urdf is not None:
        text = model_urdf
    elif simple_urdf is not None:
        text = simple_urdf
    elif asset_source.suffix == '.xacro':
        temp_path = _temp_asset_dir() / f'{asset_name}.model.source.urdf'
        _materialize_xacro_to_urdf(asset_source, temp_path)
        text = temp_path.read_text(encoding='utf-8')
    else:
        text = asset_source.read_text(encoding='utf-8')
    text = _rewrite_package_uris(text, asset_source)
    text = _convert_mesh_assets(text, asset_name)
    text = _prefix_robot_names(text, prefix)
    out = _temp_asset_dir() / f'{asset_name}.{prefix or "model"}.urdf'
    out.write_text(text, encoding='utf-8')
    return str(out)


def robot_asset_template(cfg: Config, asset_name: str) -> tuple[str, int]:
    resolved_urdf = materialize_robot_urdf(cfg, asset_name)
    meta = robot_type_metadata(cfg, asset_name)
    return resolved_urdf, int(meta['semantic_id'])


def robot_type_metadata(cfg: Config, asset_name: str) -> dict[str, float | int | str]:
    meta = dict(ROBOT_TYPE_METADATA.get(asset_name, {}))
    meta.setdefault('radius', max(float(cfg.pedestrian_radius), 0.1))
    meta.setdefault('height', max(float(cfg.pedestrian_height), 0.1))
    meta.setdefault('semantic_id', int(cfg.robot_semantic_id))
    radius_overrides = _parse_override_map(
        getattr(cfg, 'robot_radius_overrides', ''),
        cast=float,
    )
    height_overrides = _parse_override_map(
        getattr(cfg, 'robot_height_overrides', ''),
        cast=float,
    )
    semantic_overrides = _parse_override_map(
        getattr(cfg, 'robot_semantic_id_overrides', ''),
        cast=int,
    )
    if asset_name in radius_overrides:
        meta['radius'] = max(float(radius_overrides[asset_name]), 0.05)
    if asset_name in height_overrides:
        meta['height'] = max(float(height_overrides[asset_name]), 0.05)
    if asset_name in semantic_overrides:
        meta['semantic_id'] = int(semantic_overrides[asset_name])
    return meta


def robot_visual_marker_spec(cfg: Config, asset_name: str) -> dict[str, object] | None:
    asset_source = _resolve_asset_source(resolve_robot_asset_root(cfg), asset_name)
    pkg_root = asset_source.parent.parent if asset_source.parent.name == 'urdf' else asset_source.parent
    spec = _marker_spec(asset_name, pkg_root)
    if spec is None:
        return None
    if 'mesh' in spec:
        mesh_path = Path(str(spec['mesh']))
        if not mesh_path.is_file():
            return None
    elif 'parts' in spec:
        mesh_parts = [part for part in spec['parts'] if 'mesh' in part]
        if not mesh_parts:
            return None
        for part in mesh_parts:
            if not Path(str(part['mesh'])).is_file():
                return None
    else:
        return None
    return spec
