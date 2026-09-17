#!/usr/bin/env bash
# Viewer only: never start the driver, FAST-LIO, or a bag recorder.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  printf '%s\n' \
    'Usage: bash scripts/rviz.sh [mapping|localization] [--config PATH] [-- RViz arguments...]' \
    '  mapping       View /Laser_map (default); NX must pass publish_map:=true.' \
    '  localization  View transient-local /reference_map and /cloud_registered.' \
    '  --config PATH Use a custom .rviz config; relative paths follow the workspace.' \
    '  -h, --help    Show help without ROS, Docker or a display.' \
    'Loads the same DDS defaults as run.sh: Fast DDS, ROS_DOMAIN_ID=18.' \
    'Needs ROS Humble, RViz2 and a working desktop; no workspace build or PCD is required.' \
    'Docker viewing needs an existing GUI-enabled container: FASTLIO_DOCKER_GUI=1 bash docker/run.sh --detach.'
}
original_arguments=("$@")
mode=mapping
case "${1:-}" in
  mapping|localization) mode="$1"; shift ;;
  ''|-h|--help|--config|--) ;;
  *) echo "Unknown viewer mode: $1" >&2; usage >&2; exit 2 ;;
esac
custom_config=''
rviz_arguments=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --config)
      if [[ $# -lt 2 || -z "$2" || "$2" == --* ]]; then
        echo "--config requires a file path." >&2; exit 2
      fi
      custom_config="$2"; shift 2 ;;
    --) shift; rviz_arguments=("$@"); break ;;
    *) echo "Unknown viewer option: $1 (pass RViz options after --)." >&2; usage >&2; exit 2 ;;
  esac
done
source "$workspace_dir/scripts/lib/common.sh"
fastlio_dispatch rviz.sh "${original_arguments[@]}"
cd "$workspace_dir"
config="${custom_config:-$workspace_dir/src/FAST_LIO/rviz/$mode.rviz}"
if [[ ! -f "$config" ]]; then
  echo "RViz configuration not found: $config" >&2; exit 2
fi
source "$workspace_dir/scripts/setenv.bash"
if ! command -v rviz2 >/dev/null; then
  echo "RViz2 is not installed in this ROS environment (ros-humble-rviz2)." >&2; exit 2
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" && "${QT_QPA_PLATFORM:-}" != offscreen ]]; then
  echo "RViz2 needs a graphical desktop. Run on AGX's desktop or configure the container display; SSH alone is not a display." >&2; exit 2
fi
if [[ "${ROS_LOCALHOST_ONLY:-0}" == 1 ]]; then
  echo "[fastlio] WARNING: ROS_LOCALHOST_ONLY=1 prevents remote NX discovery; unset it or set it to 0." >&2
fi
exec rviz2 -d "$config" "${rviz_arguments[@]}"
