# Docker / root sessions: avoid Fast DDS SHM port lock failures and Qt runtime warnings.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

if [ -n "${XDG_RUNTIME_DIR:-}" ] && [ -d "${XDG_RUNTIME_DIR}" ]; then
  chmod 700 "${XDG_RUNTIME_DIR}" 2>/dev/null || true
fi

# Isaac Lab .bashrc aliases pip3/python3 to kit python 3.11 (broken pip). Undo for this shell.
if [ -n "${BASH_VERSION:-}" ]; then
  _autonomy_py=/opt/venv/bin/python3
  [ -x "${_autonomy_py}" ] || _autonomy_py=/usr/bin/python3
  unalias pip pip3 2>/dev/null || true
  alias pip="${_autonomy_py} -m pip"
  alias pip3="${_autonomy_py} -m pip"
  unset _autonomy_py
fi
