#!/usr/bin/env bash
# AIST Living Lab — same workflow as stella_vslam Simple Tutorial:
# https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -f "${PKG_DIR}/../../../install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${PKG_DIR}/../../../install/setup.bash"
elif [[ -f "${PKG_DIR}/../../../../install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${PKG_DIR}/../../../../install/setup.bash"
fi

DATA_ROOT="${PKG_DIR}/data/aist_living_lab"
VOCAB="${PKG_DIR}/vocab/orb_vocab.fbow"
CONFIG="${PKG_DIR}/config/atlas/aist_equirectangular.yaml"
MAP="${DATA_ROOT}/aist_living_lab_1_map.msg"

if command -v ros2 >/dev/null 2>&1; then
  RUN_VIDEO_SLAM="$(ros2 pkg prefix autonomy_slam)/lib/autonomy_slam/run_video_slam"
else
  RUN_VIDEO_SLAM="${PKG_DIR}/../../../install/autonomy_slam/lib/autonomy_slam/run_video_slam"
fi

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

usage() {
  cat <<EOF
Usage: $(basename "$0") {mapping|localization|temporal}

  mapping    aist_living_lab_1 → ${MAP}
  localization   aist_living_lab_2 with --disable-mapping
  temporal   aist_living_lab_2 with --temporal-mapping
EOF
}

case "${1:-}" in
  mapping)
    exec "${RUN_VIDEO_SLAM}" \
      -v "${VOCAB}" \
      -m "${DATA_ROOT}/aist_living_lab_1/video.mp4" \
      -c "${CONFIG}" \
      --frame-skip 3 --no-sleep \
      --map-db-out "${MAP}"
    ;;
  localization)
    exec "${RUN_VIDEO_SLAM}" \
      --disable-mapping \
      -v "${VOCAB}" \
      -m "${DATA_ROOT}/aist_living_lab_2/video.mp4" \
      -c "${CONFIG}" \
      --frame-skip 3 --no-sleep \
      --map-db-in "${MAP}"
    ;;
  temporal)
    exec "${RUN_VIDEO_SLAM}" \
      --temporal-mapping \
      -v "${VOCAB}" \
      -m "${DATA_ROOT}/aist_living_lab_2/video.mp4" \
      -c "${CONFIG}" \
      --frame-skip 3 --no-sleep \
      --map-db-in "${MAP}"
    ;;
  *)
    usage
    exit 1
    ;;
esac
