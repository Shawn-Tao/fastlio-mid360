#!/usr/bin/env bash
# Archived alias; use scripts/check_network.sh (still READ-ONLY).
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
echo "[fastlio] Read-only network check. Configure your real LiDAR NIC manually; see doc/JETSON_DEPLOY.md."
exec bash "$script_dir/check_network.sh" "$@"
