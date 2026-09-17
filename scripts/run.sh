#!/usr/bin/env bash
# Modes and launch arguments are passed without shell interpolation.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  printf '%s\n' \
    'Usage: bash scripts/run.sh [mapping|localization|replay] [--rviz|--no-rviz] [name:=value ...]' \
    '  --rviz       Start RViz2 (requires RViz2 installed and a working display).' \
    '  --no-rviz    Do not start RViz2 (the default).' \
    '  -h, --help   Show this help without starting ROS or Docker.' \
    'The existing rviz:=true / rviz:=false launch arguments remain supported.' \
    'Mapping --rviz enables bounded map display; publish_map:=true supports remote RViz.' \
    'Mapping static filtering defaults on (archive only); static_filter:=false is the legacy comparison.' \
    'Verified localization: map_metadata:=strict; CSV defaults to records/, override record_dir:=...' \
    'Compile separately: bash scripts/build.sh (no PCD required).' \
    'Mapping:      bash scripts/run.sh mapping map_name:=lab_a' \
    'Localization: bash scripts/run.sh localization map_path:=pcd_map/scene.pcd search_radius:=3.0 --rviz' \
    'Remote viewer only: bash scripts/rviz.sh mapping|localization (no driver or FAST-LIO).' \
    'Historical aliases are archived in scripts/backup/; use this unified entry point.' \
    'If RViz selections are repeated, the last selection wins.'
}
mode=mapping
case "${1:-}" in
  mapping|localization|replay) mode="$1"; shift ;;
  ''|-h|--help|--rviz|--no-rviz|*:=*) ;;
  *) echo "Unknown mode: $1" >&2; usage >&2; exit 2 ;;
esac
launch_arguments=()
rviz_argument=''
for argument in "$@"; do
  case "$argument" in
    -h|--help) usage; exit 0 ;;
    --rviz) rviz_argument='rviz:=true' ;;
    --no-rviz) rviz_argument='rviz:=false' ;;
    rviz:=*) rviz_argument="$argument" ;;
    *) launch_arguments+=("$argument") ;;
  esac
done
if [[ -n "$rviz_argument" ]]; then launch_arguments+=("$rviz_argument"); fi
source "$workspace_dir/scripts/lib/common.sh"
fastlio_dispatch run.sh "$mode" "${launch_arguments[@]}"
if [[ ! -f "$workspace_dir/install/local_setup.bash" ]]; then
  echo "Build the workspace first: bash scripts/build.sh" >&2
  exit 2
fi
source "$workspace_dir/scripts/setenv.bash"
cd "$workspace_dir"
exec ros2 launch fast_lio mid360.launch.py "mode:=$mode" "${launch_arguments[@]}"
