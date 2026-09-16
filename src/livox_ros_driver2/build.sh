#!/usr/bin/env bash
# Compatibility entry point; prefer workspace/scripts/build.sh or plain colcon.
set -eo pipefail
if [[ "${1:-humble}" != humble ]]; then
  echo "Only ROS 2 Humble is supported in this workspace." >&2
  exit 2
fi
if [[ $# -gt 0 ]]; then shift; fi
driver_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$driver_dir/../../scripts/build.sh" "$@"
