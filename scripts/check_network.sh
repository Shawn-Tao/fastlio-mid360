#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/scripts/common.sh"
fastlio_dispatch check_network.sh "$@"
ip -br addr
exec python3 "$workspace_dir/tests/check_network.py" "${1:-$workspace_dir/src/livox_ros_driver2/config/MID360.json}"
