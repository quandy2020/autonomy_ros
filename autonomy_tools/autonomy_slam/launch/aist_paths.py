"""Resolve AIST Living Lab dataset paths for launch files."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def get_aist_data_root() -> str:
  pkg_share = get_package_share_directory('autonomy_slam')
  candidates = [
      os.path.join(pkg_share, 'data', 'aist_living_lab'),
      os.path.normpath(os.path.join(
          pkg_share, '..', '..', '..', '..',
          'src', 'autonomy_ros', 'autonomy_tools', 'autonomy_slam',
          'data', 'aist_living_lab')),
  ]
  for path in candidates:
      if os.path.isdir(path):
          return path
  raise RuntimeError(
      'AIST dataset not found. Run: '
      'ros2 run autonomy_slam download_aist_example.sh '
      'or vocab/download_aist_example.sh')


def get_aist_sequence_dir(sequence: str) -> str:
  data_root = get_aist_data_root()
  seq_dir = os.path.join(data_root, sequence)
  if not os.path.isdir(seq_dir):
      raise RuntimeError(f'AIST sequence not found: {seq_dir}')
  return seq_dir


def get_aist_video(sequence: str) -> str:
  video = os.path.join(get_aist_sequence_dir(sequence), 'video.mp4')
  if not os.path.isfile(video):
      raise RuntimeError(f'Video not found: {video}')
  return video


def get_default_vocab() -> str:
  pkg_share = get_package_share_directory('autonomy_slam')
  return os.path.join(pkg_share, 'vocab', 'orb_vocab.fbow')


def get_default_atlas_config() -> str:
  pkg_share = get_package_share_directory('autonomy_slam')
  return os.path.join(pkg_share, 'config', 'atlas', 'aist_equirectangular.yaml')


def ld_library_path() -> str:
  parts = []
  if os.path.isdir('/usr/local/lib'):
      parts.append('/usr/local/lib')
  try:
      slam_prefix = get_package_prefix('autonomy_slam')
      for lib_dir in (
          os.path.join(slam_prefix, 'lib'),
          os.path.join(os.path.dirname(slam_prefix), 'autonomy', 'lib'),
      ):
          if os.path.isdir(lib_dir):
              parts.append(lib_dir)
  except Exception:
      pass
  existing = os.environ.get('LD_LIBRARY_PATH', '')
  if existing:
      parts.append(existing)
  return ':'.join(parts)

