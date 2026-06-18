# Docker / root sessions: avoid Fast DDS SHM port lock failures and Qt runtime warnings.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

if [ -n "${XDG_RUNTIME_DIR:-}" ] && [ -d "${XDG_RUNTIME_DIR}" ]; then
  chmod 700 "${XDG_RUNTIME_DIR}" 2>/dev/null || true
fi
