#!/usr/bin/env bash
# Generate host-side local files in the shared workspace; no ROS/Docker needed.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
exec python3 "$workspace_dir/scripts/lib/init_local_config.py" "$@"
