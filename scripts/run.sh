#!/usr/bin/env bash
# Modes and all additional launch arguments are passed without shell interpolation.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/scripts/common.sh"
fastlio_dispatch run.sh "$@"
if [[ ! -f "$workspace_dir/install/local_setup.bash" ]]; then
  echo "Build the workspace first: bash scripts/build.sh" >&2
  exit 2
fi
source "$workspace_dir/scripts/setenv.bash"
cd "$workspace_dir"
mode="${1:-mapping}"
if [[ $# -gt 0 ]]; then shift; fi
exec ros2 launch fast_lio mid360.launch.py "mode:=$mode" "$@"
