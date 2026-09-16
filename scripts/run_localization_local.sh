#!/usr/bin/env bash
# map_path:=pcd_map/<actual scene filename>.pcd is required by the launcher.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/run.sh" localization "$@"
