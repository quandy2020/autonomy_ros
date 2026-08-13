#!/usr/bin/env bash
# autonomy_navrl — single entry for train / demo / setup / validate / smoke / tensorboard / deps
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
AUTONOMY_ROOT="$(cd "${PKG_ROOT}/../../../.." && pwd)"
NAVRL="${SCRIPT_DIR}/navrl.sh"

# Training writes checkpoints relative to repo cwd (typically AUTONOMY_ROOT).
resolve_tensorboard_logdir() {
  local run_name="$1"
  if [[ -n "${LOGDIR:-}" ]]; then
    echo "${LOGDIR}"
    return 0
  fi
  local -a candidates=(
    "${AUTONOMY_ROOT}/checkpoints/${run_name}/tensorboard"
    "${PKG_ROOT}/checkpoints/${run_name}/tensorboard"
    "$(pwd)/checkpoints/${run_name}/tensorboard"
  )
  local dir
  for dir in "${candidates[@]}"; do
    if [[ -d "${dir}" ]]; then
      echo "${dir}"
      return 0
    fi
  done
  echo "error: tensorboard logdir not found for run '${run_name}'. Tried:" >&2
  for dir in "${candidates[@]}"; do
    echo "  - ${dir}" >&2
  done
  return 1
}

# --- Python / env -----------------------------------------------------------

navrl_python() {
  local mode="${1:-isaac}"
  if [[ "${mode}" == "mock" ]]; then
    if [[ -x /opt/venv/bin/python ]]; then echo /opt/venv/bin/python; else echo /usr/bin/python3; fi
    return
  fi
  if [[ -x /workspace/isaaclab/_isaac_sim/python.sh ]]; then
    echo /workspace/isaaclab/_isaac_sim/python.sh
  elif [[ -x /isaac-sim/python.sh ]]; then
    echo /isaac-sim/python.sh
  else
    echo /opt/venv/bin/python
  fi
}

ensure_eula() {
  export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"
  export PYTHONPATH="${PKG_ROOT}:${PYTHONPATH:-}"
}

# --- Presets & shared CLI ---------------------------------------------------

normalize_preset() {
  local name="${1,,}"
  echo "${name//_/-}"
}

# resolve_preset NAME → config|profile|assets
# assets: none | go2w-urdf | go2w-all
resolve_preset() {
  local name; name="$(normalize_preset "$1")"
  case "${name}" in
    s10|default)       echo "${PKG_ROOT}/config/s10.yaml|default|none" ;;
    s10-jit)           echo "${PKG_ROOT}/config/s10.yaml|s10-jit|s10-urdf" ;;
    jdrobot)           echo "${PKG_ROOT}/config/s10.yaml|jdrobot|none" ;;
    s10-webrtc|webrtc) echo "${PKG_ROOT}/config/s10.yaml|webrtc|none" ;;
    100envs)           echo "${PKG_ROOT}/config/s10.yaml|100envs|none" ;;
    smoke)             echo "${PKG_ROOT}/config/s10.yaml|smoke|none" ;;
    mock)              echo "${PKG_ROOT}/config/s10.yaml|mock|none" ;;
    go2w-jit|jit)      echo "${PKG_ROOT}/config/go2w.yaml|jit|go2w-all" ;;
    go2w|kinematic)    echo "${PKG_ROOT}/config/go2w.yaml|kinematic|go2w-urdf" ;;
    go2w-webrtc)       echo "${PKG_ROOT}/config/go2w.yaml|webrtc|go2w-urdf" ;;
    *) return 1 ;;
  esac
}

apply_go2w_assets() {
  case "${1:-none}" in
    go2w-all) setup_go2w_assets all ;;
    go2w-urdf) setup_go2w_assets urdf ;;
  esac
}

apply_s10_assets() {
  case "${1:-none}" in
    s10-urdf) setup_s10_urdf ;;
  esac
}

array_has_flag() {
  local flag="$1"; shift
  local arg
  for arg in "$@"; do
    [[ "${arg}" == "${flag}" ]] && return 0
  done
  return 1
}

append_if_missing() {
  local flag="$1"
  local -n _arr_ref="$2"
  local value="${3:-}"
  array_has_flag "${flag}" "${_arr_ref[@]}" && return 0
  _arr_ref+=("${flag}")
  [[ -n "${value}" ]] && _arr_ref+=("${value}")
}

# Global options (shared by train / demo / smoke / webrtc alias)
OPTS_config=""
OPTS_profile=""
OPTS_backend="${BACKEND:-isaac}"
OPTS_num_envs=""
OPTS_livestream=""
OPTS_webrtc=0
OPTS_extra=()

reset_opts() {
  OPTS_config=""
  OPTS_profile=""
  OPTS_backend="${BACKEND:-isaac}"
  OPTS_num_envs=""
  OPTS_livestream=""
  OPTS_webrtc=0
  OPTS_extra=()
}

consume_global_opt() {
  case "$1" in
    -c|--config) OPTS_config="$2"; echo 2 ;;
    -p|--profile) OPTS_profile="$2"; echo 2 ;;
    -b|--backend) OPTS_backend="$2"; echo 2 ;;
    -n|--num-envs) OPTS_num_envs="$2"; echo 2 ;;
    -w|--webrtc) OPTS_webrtc=1; OPTS_livestream="${OPTS_livestream:-2}"; echo 1 ;;
    --livestream) OPTS_livestream="$2"; echo 2 ;;
    --public-ip) export PUBLIC_IP="$2"; echo 2 ;;
    --port) export LIVESTREAM_PORT="$2"; echo 2 ;;
    *) return 1 ;;
  esac
}

is_preset_token() {
  resolve_preset "$(normalize_preset "$1")" >/dev/null 2>&1
}

is_global_opt() {
  case "$1" in
    -c|--config|-p|--profile|-b|--backend|-n|--num-envs|-w|--webrtc|--livestream|--public-ip|--port)
      return 0 ;;
  esac
  return 1
}

maybe_enable_livestream() {
  local profile="$1"
  local -n _extra_ref="$2"
  local mode=""

  if [[ "${OPTS_webrtc}" -eq 1 || -n "${OPTS_livestream}" || "${profile}" == "webrtc" ]]; then
    mode="${OPTS_livestream:-${LIVESTREAM:-2}}"
    setup_webrtc_env "${mode}"
    append_if_missing --livestream _extra_ref "${mode}"
  fi
}

build_train_cli_extra() {
  local -n _out_ref="$1"
  _out_ref=()
  [[ -n "${OPTS_num_envs}" ]] && _out_ref+=(--num-envs "${OPTS_num_envs}")
  _out_ref+=("${OPTS_extra[@]}")
}

# --- WebRTC livestream ------------------------------------------------------

detect_public_ip() {
  if [[ -n "${PUBLIC_IP:-}" ]]; then
    echo "${PUBLIC_IP}"
    return
  fi
  local ip=""
  if command -v ip >/dev/null 2>&1; then
    ip="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')"
  fi
  if [[ -z "${ip}" ]]; then
    ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  fi
  [[ -n "${ip}" ]] && echo "${ip}" || echo "127.0.0.1"
}

setup_webrtc_env() {
  local mode="${1:-${LIVESTREAM:-2}}"
  case "${mode}" in
    0|1|2) ;;
    *) echo "error: LIVESTREAM must be 0, 1, or 2 (got ${mode})" >&2; return 1 ;;
  esac

  export LIVESTREAM="${mode}"
  export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"
  export LIVESTREAM_PORT="${LIVESTREAM_PORT:-49100}"

  if [[ "${LIVESTREAM}" == "1" ]]; then
    export PUBLIC_IP="${PUBLIC_IP:-$(detect_public_ip)}"
  fi

  local mode_label="off"
  [[ "${LIVESTREAM}" == "1" ]] && mode_label="public"
  [[ "${LIVESTREAM}" == "2" ]] && mode_label="local/private"

  cat >&2 <<EOF
[webrtc] livestream=${LIVESTREAM} (${mode_label})
[webrtc] client: NVIDIA Isaac Sim WebRTC Streaming Client
[webrtc] docs:  bash scripts/navrl.sh help webrtc
EOF
  if [[ "${LIVESTREAM}" -ge 1 ]]; then
    local host="${PUBLIC_IP:-127.0.0.1}"
    echo "[webrtc] connect → ${host}:${LIVESTREAM_PORT} (Docker 需映射 -p ${LIVESTREAM_PORT}:${LIVESTREAM_PORT}/udp -p ${LIVESTREAM_PORT}:${LIVESTREAM_PORT}/tcp)" >&2
  fi
}

# --- Isaac runtime repair & preflight ---------------------------------------

resolve_isaac_root() {
  if [[ -n "${ISAAC_ROOT:-}" ]] && [[ -d "${ISAAC_ROOT}/exts" ]]; then
    echo "${ISAAC_ROOT}"; return
  fi
  for candidate in /isaac-sim /workspace/isaaclab/_isaac_sim; do
    [[ -d "${candidate}/exts" ]] || continue
    echo "${candidate}"; return
  done
}

resolve_isaac_python() {
  if [[ -n "${ISAAC_PYTHON:-}" ]] && [[ -x "${ISAAC_PYTHON}" ]]; then
    echo "${ISAAC_PYTHON}"; return
  fi
  if [[ -x /workspace/isaaclab/_isaac_sim/python.sh ]]; then
    echo /workspace/isaaclab/_isaac_sim/python.sh
  elif [[ -x /isaac-sim/python.sh ]]; then
    echo /isaac-sim/python.sh
  elif [[ -n "${ISAAC_ROOT:-}" ]] && [[ -x "${ISAAC_ROOT}/python.sh" ]]; then
    echo "${ISAAC_ROOT}/python.sh"
  fi
}

pip_install_quiet() {
  local isaac_python="$1"; shift
  PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_ROOT_USER_ACTION=ignore \
    "${isaac_python}" -m pip install --quiet "$@" 2>/dev/null
}

repair_isaac_symlinks() {
  local isaac_root isaac_python broken_link target_file source_file
  isaac_root="$(resolve_isaac_root || true)"
  [[ -n "${isaac_root}" && -d "${isaac_root}/exts" ]] || return 0

  broken_link="${isaac_root}/exts/omni.isaac.ml_archive/pip_prebundle/torch/_vendor/packaging/_structures.py"
  target_file="${isaac_root}/exts/omni.isaac.core_archive/pip_prebundle/packaging/_structures.py"
  source_file="${isaac_root}/kit/python/lib/python3.11/site-packages/packaging/_structures.py"
  if [[ -L "${broken_link}" ]] && [[ ! -e "${broken_link}" ]] && [[ -f "${source_file}" ]]; then
    mkdir -p "$(dirname "${target_file}")"
    cp -f "${source_file}" "${target_file}"
    echo "Repaired broken symlink: ${broken_link}"
  fi

  isaac_python="$(resolve_isaac_python || true)"
  [[ -n "${isaac_python}" ]] || return 0

  local numpy_major marker numpy_ver
  numpy_major="$("${isaac_python}" -c 'import numpy; print(int(numpy.__version__.split(".")[0]))' 2>/dev/null || echo 0)"
  if [[ "${numpy_major}" -ge 2 ]]; then
    marker="${isaac_root}/kit/data/.autonomy_numpy_1x"
    rm -f "${marker}"
    pip_install_quiet "${isaac_python}" "numpy==1.26.4" || {
      echo "error: failed to pin Isaac Sim NumPy to 1.26.4" >&2; return 1
    }
    numpy_ver="$("${isaac_python}" -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo unknown)"
    echo "${numpy_ver}" > "${marker}"
    echo "Pinned Isaac Sim NumPy to ${numpy_ver} (Isaac extensions require 1.x)" >&2
  fi

  local warp_ver pip_warp_dir cache_dir target_warp backup_warp patch_marker
  warp_ver="$("${isaac_python}" -c 'import warp; print(warp.__version__)' 2>/dev/null || echo 0)"
  if [[ "${warp_ver}" == 1.7.* || "${warp_ver}" == 1.7.1 || "${warp_ver}" == 0 ]]; then
    echo "Upgrading pip Warp (${warp_ver:-missing}) for Isaac compatibility ..." >&2
    pip_install_quiet "${isaac_python}" --upgrade "warp-lang>=1.8.1"
    warp_ver="$("${isaac_python}" -c 'import warp; print(warp.__version__)' 2>/dev/null || echo 0)"
  fi
  [[ "${warp_ver}" != 0 ]] || return 0
  pip_warp_dir="$("${isaac_python}" -c 'import os, warp; print(os.path.dirname(warp.__file__))' 2>/dev/null || true)"
  [[ -n "${pip_warp_dir}" && -d "${pip_warp_dir}" ]] || return 0
  for cache_dir in "${isaac_root}"/extscache/omni.warp.core-*; do
    [[ -d "${cache_dir}" ]] || continue
    target_warp="${cache_dir}/warp"
    [[ -d "${target_warp}" ]] || continue
    patch_marker="${cache_dir}/.pip_warp_patched"
    [[ -f "${patch_marker}" && "$(cat "${patch_marker}")" == "${warp_ver}" ]] && continue
    backup_warp="${cache_dir}/warp.bak.1.7.1"
    [[ -d "${backup_warp}" ]] || cp -a "${target_warp}" "${backup_warp}"
    rm -rf "${target_warp}"
    cp -a "${pip_warp_dir}" "${target_warp}"
    echo "${warp_ver}" > "${patch_marker}"
    echo "Patched extscache Warp (${cache_dir##*/}) from pip ${warp_ver}"
  done
}

preflight_isaac() {
  export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

  if [[ "${ISAAC_KILL_STALE:-0}" == "1" ]]; then
    local pattern found=0
    for pattern in '_isaac_sim/kit/python/bin/python3' '/isaac-sim/kit/python/bin/python3' '_isaac_sim/python.sh'; do
      pgrep -f "${pattern}" >/dev/null 2>&1 && found=1 && break
    done
    if [[ "${found}" -eq 1 ]]; then
      echo "Stopping stale Isaac Sim processes ..." >&2
      for pattern in '_isaac_sim/kit/python/bin/python3' '/isaac-sim/kit/python/bin/python3' '_isaac_sim/python.sh'; do
        pkill -TERM -f "${pattern}" 2>/dev/null || true
      done
      sleep 2
      for pattern in '_isaac_sim/kit/python/bin/python3' '/isaac-sim/kit/python/bin/python3' '_isaac_sim/python.sh'; do
        pkill -KILL -f "${pattern}" 2>/dev/null || true
      done
      sleep 2
    fi
  fi

  repair_isaac_symlinks

  local isaac_python warp_output
  isaac_python="$(resolve_isaac_python || true)"
  if [[ -n "${isaac_python}" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    warp_output="$("${isaac_python}" -c "import warp as wp; wp.init(); print('warp_ok', wp.__version__)" 2>&1 || true)"
    if grep -q 'cuDeviceGetUuid' <<<"${warp_output}"; then
      echo "warning: Warp still reports cuDeviceGetUuid; extscache patch may be incomplete." >&2
    fi
  fi

  if [[ "${ISAAC_FORCE:-0}" != "1" ]]; then
    local mem_avail_kb swap_total swap_free mem_avail_gb swap_used_pct=0
    mem_avail_kb="$(grep MemAvailable /proc/meminfo | awk '{print $2}')"
    swap_total="$(grep SwapTotal /proc/meminfo | awk '{print $2}')"
    swap_free="$(grep SwapFree /proc/meminfo | awk '{print $2}')"
    mem_avail_gb=$((mem_avail_kb / 1024 / 1024))
    if [[ "${swap_total}" -gt 0 ]]; then
      swap_used_pct=$(( (swap_total - swap_free) * 100 / swap_total ))
    fi
    if [[ "${mem_avail_gb}" -lt 16 || "${swap_used_pct}" -ge 80 ]]; then
      cat >&2 <<EOF
error: insufficient memory for Isaac Sim (recommend >=16GB MemAvailable, swap <80% used).
  MemAvailable: ${mem_avail_gb} GB  Swap used: ${swap_used_pct}%
  Safe: bash "${NAVRL}" smoke mock
  Force: ISAAC_FORCE=1 bash "${NAVRL}" smoke isaac
EOF
      return 1
    fi
  fi

  if [[ "${ISAAC_ALLOW_UNSUPPORTED_DRIVER:-0}" != "1" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    local driver_version major
    driver_version="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ')"
    major="${driver_version%%.*}"
    if [[ -n "${major}" && "${major}" -ge 590 ]]; then
      cat >&2 <<EOF
error: NVIDIA driver ${driver_version} is incompatible with Isaac Sim 5.x RTX rendering.
  Fix: sudo apt install nvidia-driver-580-open && sudo reboot
  Workaround: bash "${NAVRL}" smoke mock
EOF
      return 1
    fi
  fi
}

# --- Go2W assets ------------------------------------------------------------

is_lfs_pointer() {
  [[ -f "$1" ]] && head -1 "$1" | grep -q "git-lfs"
}

setup_go2w_urdf() {
  local dst="${PKG_ROOT}/urdf/go2w_description"
  local vendor="${dst}/vendor/unitree_ros"
  local unitree_repo="${UNITREE_GO2W_REPO:-https://github.com/unitreerobotics/unitree_ros.git}"
  local src meshes_dir urdf_src

  resolve_isaac_go2w_src() {
    local candidate
    if [[ -n "${GO2W_ASSETS_SRC:-}" && -d "${GO2W_ASSETS_SRC}" ]]; then echo "${GO2W_ASSETS_SRC}"; return; fi
    for candidate in \
      "${AUTONOMY_ROOT}/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description" \
      "/workspace/autonomy/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description" \
      "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description"; do
      [[ -d "${candidate}/urdf" ]] && echo "${candidate}" && return
    done
    return 1
  }

  ensure_unitree_vendor() {
    if [[ -f "${vendor}/robots/go2w_description/dae/base.dae" ]] \
       && ! is_lfs_pointer "${vendor}/robots/go2w_description/dae/base.dae"; then
      return 0
    fi
    echo "[setup] Isaac Lab meshes unavailable — cloning unitree_ros go2w_description ..." >&2
    rm -rf "${vendor}"
    git clone --depth 1 --filter=blob:none --sparse "${unitree_repo}" "${vendor}" >&2
    git -C "${vendor}" sparse-checkout set robots/go2w_description >&2
    is_lfs_pointer "${vendor}/robots/go2w_description/dae/base.dae" && {
      echo "error: unitree_ros Go2W meshes still missing after clone" >&2; exit 1
    }
  }

  src="$(resolve_isaac_go2w_src || true)"
  if [[ -n "${src}" && -f "${src}/meshes/base.dae" ]] && ! is_lfs_pointer "${src}/meshes/base.dae"; then
    :
  else
    ensure_unitree_vendor
    src="${vendor}/robots/go2w_description"
  fi

  if [[ -d "${src}/meshes" ]]; then
    meshes_dir="${src}/meshes"; urdf_src="${src}/urdf/go2w_description.urdf"
  elif [[ -d "${src}/dae" ]]; then
    meshes_dir="${src}/dae"; urdf_src="${src}/urdf/go2w_description.urdf"
  else
    echo "error: no meshes/ or dae/ under ${src}" >&2; exit 1
  fi

  mkdir -p "${dst}/urdf"
  ln -sfn "${meshes_dir}" "${dst}/meshes"
  [[ -d "${src}/usd" ]] && ln -sfn "${src}/usd" "${dst}/usd"
  local meshes_abs; meshes_abs="$(cd "${dst}/meshes" && pwd)"
  sed -e "s|package://go2w_description/meshes/|${meshes_abs}/|g" \
      -e "s|package://go2w_description/dae/|${meshes_abs}/|g" \
      "${urdf_src}" > "${dst}/urdf/go2w_description.urdf"

  python3 - <<PY
from pathlib import Path
urdf_path = Path("${dst}/urdf/go2w_description.urdf")
text = urdf_path.read_text(encoding="utf-8")
if "camera_optical_frame" not in text:
    snippet = """
  <link name="camera_optical_frame">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.001" />
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>
  <joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707963268 0 -1.5707963268" />
    <parent link="radar" />
    <child link="camera_optical_frame" />
  </joint>
"""
    text = text.replace("</robot>", snippet + "</robot>", 1)
    urdf_path.write_text(text, encoding="utf-8")
PY
  echo "[setup] Go2W URDF: ${dst}/urdf/go2w_description.urdf"
}

setup_go2w_policy() {
  local weights_dir="${PKG_ROOT}/weights/go2w" min_bytes=10000
  local candidates=(
    "${AUTONOMY_ROOT}/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description/policy.pt"
    "${AUTONOMY_ROOT}/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description/unitree_go2w_flat.pt"
    "/workspace/autonomy/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description/policy.pt"
    "/workspace/autonomy/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description/unitree_go2w_flat.pt"
    "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description/policy.pt"
    "/workspace/isaaclab/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/unitree_go2w/assets/go2w_description/unitree_go2w_flat.pt"
  )
  local path src dest size

  is_real_policy() {
    [[ -f "$1" ]] || return 1
    size="$(stat -c%s "$1" 2>/dev/null || stat -f%z "$1")"
    [[ "${size}" -ge "${min_bytes}" ]]
  }

  if [[ -d "${AUTONOMY_ROOT}/IsaacLab/.git" ]] && command -v git >/dev/null 2>&1; then
    (cd "${AUTONOMY_ROOT}/IsaacLab" && git lfs pull --include='**/unitree_go2w/**' 2>/dev/null) || true
  fi

  src=""
  for path in "${candidates[@]}"; do
    is_real_policy "${path}" && src="${path}" && break
  done
  if [[ -z "${src}" ]]; then
    echo "error: Go2W JIT policy not found (Git LFS stub?). Run: bash ${NAVRL} setup go2w --fetch" >&2
    exit 1
  fi

  mkdir -p "${weights_dir}"
  dest="${weights_dir}/policy.pt"
  cp -f "${src}" "${dest}"
  echo "[OK] Go2W policy: ${dest} ($(stat -c%s "${dest}" 2>/dev/null || stat -f%z "${dest}") bytes)"
}

fetch_go2w_policy() {
  local isaaclab_dir="${ISAACLAB_DIR:-${AUTONOMY_ROOT}/IsaacLab}"
  local git_lfs_ver="${GIT_LFS_VERSION:-3.4.1}" install_dir="${HOME}/.local/git-lfs"

  if ! command -v git-lfs >/dev/null 2>&1; then
    if [[ -x "${install_dir}/git-lfs" ]]; then
      export PATH="${install_dir}:${PATH}"
    else
      echo "[fetch] installing git-lfs to ${install_dir} (no sudo) ..." >&2
      mkdir -p "${install_dir}"
      local tmp archive
      tmp="$(mktemp -d)"; trap 'rm -rf "${tmp}"' RETURN
      archive="git-lfs-linux-amd64-v${git_lfs_ver}.tar.gz"
      curl -fsSL "https://github.com/git-lfs/git-lfs/releases/download/v${git_lfs_ver}/${archive}" -o "${tmp}/${archive}"
      tar -xzf "${tmp}/${archive}" -C "${tmp}"
      install -m 755 "${tmp}/git-lfs-${git_lfs_ver}/git-lfs" "${install_dir}/git-lfs"
      export PATH="${install_dir}:${PATH}"
      git lfs install --skip-repo
    fi
  fi

  [[ -d "${isaaclab_dir}/.git" ]] || {
    echo "error: IsaacLab repo not found at ${isaaclab_dir}" >&2; exit 1
  }
  (cd "${isaaclab_dir}" && git lfs pull --include='**/unitree_go2w/**')
  setup_go2w_policy
}

setup_go2w_assets() {
  case "${1:-all}" in
    urdf) setup_go2w_urdf ;;
    policy) setup_go2w_policy ;;
    all) setup_go2w_urdf; setup_go2w_policy ;;
    *) echo "error: setup part must be urdf|policy|all" >&2; return 1 ;;
  esac
}

setup_s10_urdf() {
  local urdf="${PKG_ROOT}/urdf/S10-urdf-2/urdf/S10.urdf"
  [[ -f "${urdf}" ]] || { echo "error: S10 URDF not found: ${urdf}" >&2; exit 1; }
  python3 - <<PY
from pathlib import Path
urdf_path = Path("${urdf}")
text = urdf_path.read_text(encoding="utf-8")
if "camera_link" not in text:
    snippet = """
  <link name="camera_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.001" />
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <origin xyz="0.12 0.0 0.08" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="camera_link" />
  </joint>
  <link name="camera_optical_frame">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.001" />
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>
  <joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707963268 0 -1.5707963268" />
    <parent link="camera_link" />
    <child link="camera_optical_frame" />
  </joint>
"""
    text = text.replace("</robot>", snippet + "</robot>")
    urdf_path.write_text(text, encoding="utf-8")
    print(f"[setup] injected camera links into {urdf_path}")
else:
    print(f"[setup] S10 URDF OK: {urdf_path}")
PY
}

setup_s10_assets() {
  case "${1:-urdf}" in
    urdf) setup_s10_urdf ;;
    *) echo "error: setup s10 part must be urdf" >&2; return 1 ;;
  esac
}

# --- Commands ---------------------------------------------------------------

usage() {
  cat <<'EOF'
Usage: bash scripts/navrl.sh <command> [preset] [options] [train_cli...]

Commands:
  train [preset]   训练（preset 默认 s10）
  demo [go2w|s10]  精准对齐 demo（上层脚本控制器 + 低层 JIT；默认 go2w）
  demo-batch go2w  随机起点 × N 次重复测试，保存 Isaac 视口 MP4（默认第三人称视角）
  demo-s10-loco    S10 运控冒烟（s10_gait：踏步/横移/前进后退/转弯）
  train-s10-loco   Isaac Lab 训练 S10 低层速度策略并导出 JIT
  eval go2w        加载 checkpoint 在 Isaac 中评估（配合 -w 可 WebRTC）
  setup go2w|s10   准备机器人资源（go2w: --fetch|--urdf|--policy；s10: urdf）
  validate [go2w]  配置与插件校验
  smoke [mock|isaac|webrtc]
  tensorboard      -r RUN_NAME  [--port N] [--restart]
  deps [deploy|train]
  clean            清除 autonomy_navrl 的 colcon 构建缓存（配置重构后若 build 失败请执行）
  help [webrtc|memory|presets]

Global options (train / demo / smoke):
  -c, --config PATH       配置文件
  -p, --profile NAME      profile 名
  -b, --backend isaac|mock
  -n, --num-envs N        并行环境数
  -w, --webrtc            WebRTC 推流（livestream=2，自动选 webrtc profile）
  --livestream 0|1|2      显式推流模式
  --public-ip IP          公网推流地址（mode=1）

Presets:
  s10  s10-jit  default  jdrobot  webrtc  100envs  smoke  mock
  go2w-jit  go2w  go2w-webrtc  (aliases: jit, kinematic)

Examples:
  bash scripts/navrl.sh setup s10 urdf
  bash scripts/navrl.sh train-s10-loco --headless
  bash scripts/navrl.sh demo-s10-loco --vx 0.3
  bash scripts/navrl.sh demo s10 -w -x 1 -y 0 --yaw 0
  bash scripts/navrl.sh train s10-jit -n 16
  bash scripts/navrl.sh train go2w-jit -n 16
  bash scripts/navrl.sh train go2w -w
  bash scripts/navrl.sh train s10 -w
  bash scripts/navrl.sh demo -w -x 2 -y 0 --yaw 45
  bash scripts/navrl.sh demo-batch go2w --trials 5 --seed 0 --video-dir /workspace/autonomy/checkpoints/go2w_batch_demo/videos
  bash scripts/navrl.sh eval go2w -w --checkpoint checkpoints/navrl_go2w/ckpts/navrl_final.pt
  bash scripts/navrl.sh setup go2w --fetch
  bash scripts/navrl.sh smoke webrtc
EOF
}

usage_presets() {
  cat <<EOF
Preset registry (config | profile | assets):

  s10 / default     s10.yaml | default
  jdrobot           s10.yaml | jdrobot
  webrtc / s10-webrtc s10.yaml | webrtc
  100envs           s10.yaml | 100envs
  smoke / mock      s10.yaml | smoke / mock
  go2w-jit / jit    go2w.yaml | jit        (+ policy)
  go2w / kinematic  go2w.yaml | kinematic  (+ urdf)
  go2w-webrtc       go2w.yaml | webrtc      (+ urdf)
  s10-jit           s10.yaml | s10-jit      (+ urdf)

WebRTC 简写: train <go2w|s10> -w   （等同 webrtc profile + livestream）
EOF
}

cmd_train() {
  reset_opts
  local preset="" config="" profile="" assets="none"
  local -a passthrough=()
  local consumed=0

  while [[ $# -gt 0 ]]; do
    if [[ -z "${preset}" ]] && is_preset_token "$1"; then
      preset="$(normalize_preset "$1")"; shift; continue
    fi
    if is_global_opt "$1"; then
      consumed="$(consume_global_opt "$1" "${2:-}")"
      shift "${consumed}"
      continue
    fi
    passthrough+=("$1"); shift
  done
  OPTS_extra+=("${passthrough[@]}")

  if [[ -n "${OPTS_config}" ]]; then
    config="${OPTS_config}"
    profile="${OPTS_profile}"
  else
    preset="${preset:-s10}"
    local resolved
    resolved="$(resolve_preset "${preset}")" || {
      echo "error: unknown preset '${preset}' (see: bash scripts/navrl.sh help presets)" >&2
      exit 1
    }
    IFS='|' read -r config profile assets <<< "${resolved}"
    profile="${OPTS_profile:-${profile}}"
  fi
  [[ -f "${config}" ]] || { echo "error: config not found: ${config}" >&2; exit 1; }

  if [[ "${OPTS_webrtc}" -eq 1 && "${profile}" != "webrtc" ]]; then
    profile="webrtc"
  fi

  case "${assets}" in
    go2w-all|go2w-urdf) apply_go2w_assets "${assets}" ;;
    s10-urdf) apply_s10_assets urdf ;;
  esac
  local -a extra=()
  build_train_cli_extra extra
  maybe_enable_livestream "${profile}" extra

  ensure_eula
  local python; python="$(navrl_python "${OPTS_backend}")"
  [[ "${OPTS_backend}" == "isaac" ]] && preflight_isaac

  echo "[navrl] train preset=${preset:-custom} config=${config} profile=${profile} backend=${OPTS_backend}"
  exec "${python}" -u -m autonomy_navrl.train.cli \
    --config "${config}" --backend "${OPTS_backend}" \
    ${profile:+--profile "${profile}"} \
    "${extra[@]}"
}

cmd_demo() {
  reset_opts
  local robot="go2w"
  local target_x="" target_y="" target_yaw="" max_steps="" headless=1
  local consumed=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      go2w|s10) robot="$1"; shift ;;
      -x|--target-x) target_x="$2"; shift 2 ;;
      -y|--target-y) target_y="$2"; shift 2 ;;
      --yaw|--target-yaw-deg) target_yaw="$2"; shift 2 ;;
      --max-steps) max_steps="$2"; shift 2 ;;
      --headless) headless=1; shift ;;
      --no-headless) headless=0; shift ;;
      *)
        if is_global_opt "$1"; then
          consumed="$(consume_global_opt "$1" "${2:-}")"
          shift "${consumed}"
        else
          OPTS_extra+=("$1"); shift
        fi
        ;;
    esac
  done

  local -a extra=()
  build_train_cli_extra extra
  maybe_enable_livestream "demo" extra
  [[ "${OPTS_webrtc}" -eq 1 || -n "${OPTS_livestream}" ]] && preflight_isaac

  ensure_eula
  local python; python="$(navrl_python isaac)"
  local -a args=()
  if [[ "${robot}" == "s10" ]]; then
    setup_s10_urdf
    args=(--config "${PKG_ROOT}/config/s10.yaml" --profile demo)
    echo "[navrl] demo s10 precision (s10_jit + pose tracker)"
  else
    apply_go2w_assets go2w-all 2>/dev/null || apply_go2w_assets go2w-urdf
    args=(--config "${PKG_ROOT}/config/go2w.yaml" --profile demo)
    echo "[navrl] demo go2w precision (go2w_jit + pose tracker)"
  fi
  [[ -n "${target_x}" ]] && args+=(--target-x "${target_x}")
  [[ -n "${target_y}" ]] && args+=(--target-y "${target_y}")
  [[ -n "${target_yaw}" ]] && args+=(--target-yaw-deg "${target_yaw}")
  [[ -n "${max_steps}" ]] && args+=(--max-steps "${max_steps}")
  [[ "${headless}" -eq 1 ]] && args+=(--headless)
  args+=("${extra[@]}")
  if [[ "${robot}" == "s10" ]]; then
    exec "${python}" -m autonomy_navrl.demo.s10_precision_demo "${args[@]}"
  else
    exec "${python}" -m autonomy_navrl.demo.go2w_precision_demo "${args[@]}"
  fi
}

cmd_demo_batch() {
  reset_opts
  local target_x="" target_y="" target_yaw="" trials=5 seed=0 video_dir="" max_steps=""
  local consumed=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      go2w) shift ;;
      --trials) trials="$2"; shift 2 ;;
      --seed) seed="$2"; shift 2 ;;
      --video-dir) video_dir="$2"; shift 2 ;;
      -x|--target-x) target_x="$2"; shift 2 ;;
      -y|--target-y) target_y="$2"; shift 2 ;;
      --yaw|--target-yaw-deg) target_yaw="$2"; shift 2 ;;
      --max-steps) max_steps="$2"; shift 2 ;;
      *)
        if is_global_opt "$1"; then
          consumed="$(consume_global_opt "$1" "${2:-}")"
          shift "${consumed}"
        else
          OPTS_extra+=("$1"); shift
        fi
        ;;
    esac
  done

  local -a extra=()
  build_train_cli_extra extra
  maybe_enable_livestream "demo" extra
  [[ "${OPTS_webrtc}" -eq 1 || -n "${OPTS_livestream}" ]] && preflight_isaac

  ensure_eula
  apply_go2w_assets go2w-all 2>/dev/null || apply_go2w_assets go2w-urdf

  local python; python="$(navrl_python isaac)"
  local -a args=(
    --config "${PKG_ROOT}/config/go2w.yaml"
    --profile demo
    --trials "${trials}"
    --seed "${seed}"
  )
  [[ -n "${video_dir}" ]] && args+=(--video-dir "${video_dir}")
  [[ -n "${target_x}" ]] && args+=(--target-x "${target_x}")
  [[ -n "${target_y}" ]] && args+=(--target-y "${target_y}")
  [[ -n "${target_yaw}" ]] && args+=(--target-yaw-deg "${target_yaw}")
  [[ -n "${max_steps}" ]] && args+=(--max-steps "${max_steps}")
  args+=("${extra[@]}")
  echo "[navrl] demo-batch trials=${trials} seed=${seed} backend=isaac"
  exec "${python}" -m autonomy_navrl.demo.go2w_batch_demo "${args[@]}"
}

cmd_eval() {
  reset_opts
  local checkpoint="" target_x="" target_y="" target_yaw="" max_steps="" profile="demo"
  local consumed=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      go2w) shift ;;
      --checkpoint|-C) checkpoint="$2"; shift 2 ;;
      -x|--target-x) target_x="$2"; shift 2 ;;
      -y|--target-y) target_y="$2"; shift 2 ;;
      --yaw|--target-yaw-deg) target_yaw="$2"; shift 2 ;;
      --max-steps) max_steps="$2"; shift 2 ;;
      -p|--profile) profile="$2"; shift 2 ;;
      *)
        if is_global_opt "$1"; then
          consumed="$(consume_global_opt "$1" "${2:-}")"
          shift "${consumed}"
        else
          OPTS_extra+=("$1"); shift
        fi
        ;;
    esac
  done

  if [[ -z "${checkpoint}" ]]; then
    local default_ckpt="${AUTONOMY_ROOT}/checkpoints/navrl_go2w/ckpts/navrl_final.pt"
    if [[ -f "${default_ckpt}" ]]; then
      checkpoint="${default_ckpt}"
      echo "[navrl] eval: using default checkpoint ${checkpoint}"
    else
      echo "error: --checkpoint required (e.g. checkpoints/navrl_go2w/ckpts/navrl_final.pt)" >&2
      exit 1
    fi
  fi

  if [[ "${OPTS_webrtc}" -eq 1 ]]; then
    profile="webrtc"
  fi

  local -a extra=()
  build_train_cli_extra extra
  maybe_enable_livestream "${profile}" extra
  [[ "${OPTS_webrtc}" -eq 1 || -n "${OPTS_livestream}" ]] && preflight_isaac

  ensure_eula
  apply_go2w_assets go2w-all 2>/dev/null || apply_go2w_assets go2w-urdf

  local python; python="$(navrl_python isaac)"
  local -a args=(
    --config "${PKG_ROOT}/config/go2w.yaml"
    --profile "${profile}"
    --checkpoint "${checkpoint}"
  )
  [[ -n "${target_x}" ]] && args+=(--target-x "${target_x}")
  [[ -n "${target_y}" ]] && args+=(--target-y "${target_y}")
  [[ -n "${target_yaw}" ]] && args+=(--target-yaw-deg "${target_yaw}")
  [[ -n "${max_steps}" ]] && args+=(--max-steps "${max_steps}")
  args+=("${extra[@]}")
  echo "[navrl] eval profile=${profile} checkpoint=${checkpoint} backend=isaac"
  exec "${python}" -m autonomy_navrl.demo.go2w_policy_eval "${args[@]}"
}

cmd_setup() {
  local robot="${1:-go2w}"
  shift || true
  case "${robot}" in
    go2w)
      local part="all"
      while [[ $# -gt 0 ]]; do
        case "$1" in
          --fetch) fetch_go2w_policy; return ;;
          --urdf|urdf) part="urdf" ;;
          --policy|policy) part="policy" ;;
          all) part="all" ;;
          *) echo "error: unknown setup option: $1" >&2; exit 1 ;;
        esac
        shift
      done
      setup_go2w_assets "${part}"
      ;;
    s10)
      local part="urdf"
      while [[ $# -gt 0 ]]; do
        case "$1" in
          --urdf|urdf) part="urdf" ;;
          *) echo "error: setup s10 supports: urdf" >&2; exit 1 ;;
        esac
        shift
      done
      setup_s10_assets "${part}"
      ;;
    *) echo "error: setup supports go2w | s10" >&2; exit 1 ;;
  esac
}

cmd_train_s10_loco() {
  reset_opts
  local -a passthrough=()
  local consumed=0
  while [[ $# -gt 0 ]]; do
    if is_global_opt "$1"; then
      consumed="$(consume_global_opt "$1" "${2:-}")"
      shift "${consumed}"
    else
      # Isaac Lab RSL-RL flags (--num_envs / --max_iterations / --headless / --resume …)
      passthrough+=("$1"); shift
    fi
  done
  local -a extra=()
  build_train_cli_extra extra
  # Preserve Isaac Lab CLI after global opts (build_train_cli_extra resets the array).
  extra+=("${passthrough[@]}")
  # Map -n/--num-envs into Isaac Lab --num_envs when not already provided.
  if [[ -n "${OPTS_num_envs}" ]]; then
    local has_num=0
    local a
    for a in "${extra[@]}"; do
      [[ "${a}" == "--num_envs" || "${a}" == "--num-envs" ]] && has_num=1 && break
    done
    if [[ "${has_num}" -eq 0 ]]; then
      extra+=(--num_envs "${OPTS_num_envs}")
    fi
  fi
  maybe_enable_livestream "s10-loco" extra
  ensure_eula
  preflight_isaac || exit 1
  setup_s10_urdf
  bash "${PKG_ROOT}/scripts/train_s10_locomotion.sh" "${extra[@]}"
}

cmd_demo_s10_loco() {
  reset_opts
  local vx="" vy="" w="" max_steps="" gait_mode="" headless=1 hold_stance=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --vx) vx="$2"; shift 2 ;;
      --vy) vy="$2"; shift 2 ;;
      --w) w="$2"; shift 2 ;;
      --max-steps) max_steps="$2"; shift 2 ;;
      --gait-mode) gait_mode="$2"; shift 2 ;;
      --hold-stance) hold_stance=1; shift ;;
      --headless) headless=1; shift ;;
      --no-headless) headless=0; shift ;;
      *)
        if is_global_opt "$1"; then
          consumed="$(consume_global_opt "$1" "${2:-}")"
          shift "${consumed}"
        else
          OPTS_extra+=("$1"); shift
        fi
        ;;
    esac
  done
  local -a extra=()
  build_train_cli_extra extra
  maybe_enable_livestream "s10-loco" extra
  ensure_eula
  preflight_isaac || exit 1
  setup_s10_urdf
  local python; python="$(navrl_python isaac)"
  local -a args=(--config "${PKG_ROOT}/config/s10.yaml" --profile s10-loco)
  [[ -n "${vx}" ]] && args+=(--vx "${vx}")
  [[ -n "${vy}" ]] && args+=(--vy "${vy}")
  [[ -n "${w}" ]] && args+=(--w "${w}")
  [[ -n "${max_steps}" ]] && args+=(--max-steps "${max_steps}")
  [[ -n "${gait_mode}" ]] && args+=(--gait-mode "${gait_mode}")
  [[ "${headless}" -eq 1 ]] && args+=(--headless)
  [[ "${hold_stance}" -eq 1 ]] && args+=(--hold-stance)
  args+=("${extra[@]}")
  echo "[navrl] demo-s10-loco backend=isaac"
  exec "${python}" -m autonomy_navrl.demo.s10_locomotion_demo "${args[@]}"
}

cmd_validate() {
  local scope="${1:-go2w}"
  ensure_eula
  local python; python="$(navrl_python isaac)"

  echo "=== list-plugins ==="
  "${python}" -m autonomy_navrl.train.cli --list-plugins
  echo ""
  echo "=== dry-run go2w.yaml (jit) ==="
  "${python}" -m autonomy_navrl.train.cli \
    --config "${PKG_ROOT}/config/go2w.yaml" --profile jit --dry-run

  if [[ "${scope}" == "go2w" ]]; then
    echo ""
    echo "=== Go2W URDF ==="
    setup_go2w_assets urdf 2>&1 | tail -3
    local policy="${PKG_ROOT}/weights/go2w/policy.pt"
    if [[ -f "${policy}" ]] && [[ $(stat -c%s "${policy}" 2>/dev/null || echo 0) -ge 10000 ]]; then
      echo "[OK] policy: ${policy}"
    else
      echo "[WARN] policy missing — bash ${NAVRL} setup go2w --fetch"
    fi
  fi
  echo "[OK] validate complete"
}

cmd_webrtc() {
  local robot="${1:-s10}"
  shift || true
  case "$(normalize_preset "${robot}")" in
    go2w|go2w-webrtc|kinematic) cmd_train go2w -w "$@" ;;
    s10|default|*) cmd_train s10 -w "$@" ;;
  esac
}

cmd_smoke() {
  local mode="${1:-isaac}"
  shift || true
  ensure_eula
  export PYTHONUNBUFFERED=1
  case "${mode}" in
    mock) cmd_train mock -b mock --total_timesteps 4 "$@" ;;
    isaac)
      preflight_isaac || {
        [[ "${SMOKE_FALLBACK_MOCK:-0}" == "1" ]] || exit 1
        echo "[navrl] Isaac preflight failed; falling back to mock" >&2
        cmd_smoke mock "$@"; exit 0
      }
      cmd_train smoke -b isaac "$@"
      ;;
    webrtc)
      preflight_isaac || exit 1
      cmd_train s10 -w -n 1 "$@"
      ;;
    *) echo "error: smoke mode must be mock|isaac|webrtc" >&2; exit 1 ;;
  esac
}

cmd_tensorboard() {
  local run_name="" port="6006" restart=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -r|--run-name) run_name="$2"; shift 2 ;;
      --port) port="$2"; shift 2 ;;
      --restart) restart=1; shift ;;
      *) shift ;;
    esac
  done
  run_name="${run_name:-navrl_go2w_jit}"
  local logdir
  logdir="$(resolve_tensorboard_logdir "${run_name}")" || exit 1
  local python="${ISAAC_PYTHON:-$(navrl_python isaac 2>/dev/null || echo python3)}"
  if pgrep -f "tensorboard.main.*--port ${port}" >/dev/null 2>&1; then
    if [[ "${restart}" -eq 1 ]]; then
      pkill -f "tensorboard.main.*--port ${port}" 2>/dev/null || true; sleep 1
    else
      echo "[tensorboard] http://localhost:${port}  logdir=${logdir}"; exit 0
    fi
  fi
  echo "[tensorboard] logdir=${logdir} port=${port}"
  exec "${python}" -m tensorboard.main --logdir "${logdir}" --bind_all --port "${port}"
}

cmd_deps() {
  local kind="${1:-deploy}"
  local python="${PYTHON:-/usr/bin/python3}"
  [[ -x /opt/venv/bin/python ]] && python="${TRAIN_PYTHON:-/opt/venv/bin/python}"
  export PYTHONPATH=
  "${python}" -m pip install --upgrade 'pip<26' 'setuptools>=30.3.0,<80'
  case "${kind}" in
    deploy) "${python}" -m pip install -r "${PKG_ROOT}/requirements.txt" ;;
    train)
      "${python}" -m pip install -r "${PKG_ROOT}/requirements-train.txt"
      repair_isaac_symlinks
      ;;
    *) echo "error: deps kind must be deploy|train" >&2; exit 1 ;;
  esac
}

cmd_clean() {
  # Remove stale symlink-install artifacts after config/ file renames or deletions.
  local ws="${AUTONOMY_ROOT:-$(cd "${PKG_ROOT}/../../../.." && pwd)}"
  local targets=(
    "${ws}/build/autonomy_navrl"
    "${ws}/install/autonomy_navrl"
  )
  echo "[clean] removing colcon artifacts for autonomy_navrl ..."
  for dir in "${targets[@]}"; do
    if [[ -e "${dir}" ]]; then
      rm -rf "${dir}"
      echo "[clean] removed ${dir}"
    fi
  done
  echo "[clean] done. Rebuild: colcon build --packages-select autonomy_navrl --symlink-install"
}

help_topic() {
  case "${1:-}" in
    presets) usage_presets ;;
    webrtc) cat <<EOF
WebRTC 推流（Isaac Sim livestream）

推荐用法（统一 -w 标志）:
  bash scripts/navrl.sh train go2w -w
  bash scripts/navrl.sh train s10 -w
  bash scripts/navrl.sh demo -w
  bash scripts/navrl.sh smoke webrtc

公网推流:
  bash scripts/navrl.sh train s10 -w --livestream 1 --public-ip 192.168.1.10

客户端: NVIDIA Isaac Sim WebRTC Streaming Client
  https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html

环境变量: LIVESTREAM (0|1|2)  PUBLIC_IP  LIVESTREAM_PORT (默认 49100)
Docker: -p 49100:49100/udp -p 49100:49100/tcp
EOF
    ;;
    memory) cat <<EOF
内存不足: free -h && swapon --show
  bash scripts/navrl.sh smoke mock
  ISAAC_FORCE=1 bash scripts/navrl.sh smoke isaac
EOF
    ;;
    *) usage ;;
  esac
}

main() {
  local cmd="${1:-help}"; shift || true
  case "${cmd}" in
    train) cmd_train "$@" ;;
    demo) cmd_demo "$@" ;;
    demo-batch) shift; cmd_demo_batch go2w "$@" ;;
    demo-s10-loco) cmd_demo_s10_loco "$@" ;;
    train-s10-loco) cmd_train_s10_loco "$@" ;;
    eval) cmd_eval "$@" ;;
    webrtc) cmd_webrtc "$@" ;;  # alias: train <robot> -w
    setup) cmd_setup "$@" ;;
    validate) cmd_validate "${1:-go2w}" ;;
    smoke) cmd_smoke "$@" ;;
    tensorboard|tb) cmd_tensorboard "$@" ;;
    deps) cmd_deps "$@" ;;
    clean) cmd_clean "$@" ;;
    help|-h|--help) help_topic "${1:-}" ;;
    *) echo "error: unknown command: ${cmd}" >&2; usage; exit 1 ;;
  esac
}

main "$@"
