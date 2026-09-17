#!/usr/bin/env bash
# Internal dispatch helper; callers set workspace_dir from their own location.
fastlio_dispatch() {
  local script_name="$1"
  shift
  case "${FASTLIO_NATIVE:-auto}" in
    1) return 0 ;;
    0) exec bash "$workspace_dir/scripts/lib/in_container.sh" "$script_name" "$@" ;;
    auto) ;;
    *) echo "FASTLIO_NATIVE must be auto, 0 (Docker), or 1 (native)." >&2; return 2 ;;
  esac
  if [[ -f /.dockerenv ]]; then return 0; fi
  if [[ -n "${FASTLIO_CONTAINER:-}" || ! -f /opt/ros/humble/setup.bash ]]; then
    exec bash "$workspace_dir/scripts/lib/in_container.sh" "$script_name" "$@"
  fi
}
