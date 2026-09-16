#!/usr/bin/env bash
# Old name retained, now READ-ONLY: no fixed NIC, address or sudo side effect.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
echo "[fastlio] Read-only network check. Configure your real LiDAR NIC manually; see doc/JETSON_DEPLOY.md."
exec bash "$script_dir/check_network.sh" "$@"
