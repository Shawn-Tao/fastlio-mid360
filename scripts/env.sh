# Source from Bash or Zsh: source /path/to/fastlio-mid360_space/scripts/env.sh
# Running a child shell cannot change the environment of the current terminal.
if [ -n "${BASH_VERSION:-}" ]; then
  if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    printf '%s\n' 'Use: source scripts/env.sh (not bash scripts/env.sh).' >&2
    exit 2
  fi
elif [ -n "${ZSH_VERSION:-}" ]; then
  case "$ZSH_EVAL_CONTEXT" in
    *:file) ;;
    *) printf '%s\n' 'Use: source scripts/env.sh (not zsh scripts/env.sh).' >&2; exit 2 ;;
  esac
else
  printf '%s\n' 'scripts/env.sh must be sourced from Bash or Zsh.' >&2
  return 2 2>/dev/null || exit 2
fi

_fastlio_env_load() {
  local fastlio_env_entry fastlio_env_shell fastlio_env_dir
  if [ -n "${BASH_VERSION:-}" ]; then
    fastlio_env_entry="${BASH_SOURCE[0]}"
    fastlio_env_shell=bash
  else
    fastlio_env_entry="${(%):-%x}"
    fastlio_env_shell=zsh
  fi
  fastlio_env_dir="$(cd -- "$(dirname -- "$fastlio_env_entry")/.." && pwd)" || return 2
  source "$fastlio_env_dir/scripts/setenv.$fastlio_env_shell" || {
    printf '%s\n' '[fastlio] Failed to load ROS/workspace/DDS environment; check the errors above.' >&2
    return 2
  }

  printf '[fastlio] workspace=%s\n' "$fastlio_env_dir"
  printf '[fastlio] ROS_DISTRO=%s ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s ROS_LOCALHOST_ONLY=%s\n' \
    "${ROS_DISTRO:-humble}" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION" "${ROS_LOCALHOST_ONLY:-0}"
  if [ "${ROS_LOCALHOST_ONLY:-0}" = 1 ]; then
    printf '%s\n' '[fastlio] WARNING: ROS_LOCALHOST_ONLY=1 prevents NX/AGX cross-machine discovery.' >&2
  fi
  printf '%s\n' '[fastlio] Environment loaded in this terminal. Try: ros2 topic list'
}

if _fastlio_env_load; then
  unset -f _fastlio_env_load
else
  unset -f _fastlio_env_load
  return 2
fi
