#!/usr/bin/env bash
# Local setup only: no sudo, system Python, shell profiles or ROS overlay.
set -euo pipefail
eval_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$eval_dir/../.." && pwd)"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo "This open3d-cpu environment is for the x86_64 Linux evaluation PC, not Jetson." >&2
  exit 2
fi
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV
export UV_CACHE_DIR="$workspace_dir/.local_tools/cache"
export UV_PYTHON_INSTALL_DIR="$workspace_dir/.local_tools/python"
export UV_PYTHON_INSTALL_BIN=0
export UV_PROJECT_ENVIRONMENT="$eval_dir/.venv"
export UV_NO_MODIFY_PATH=1
export PYTHONNOUSERSITE=1
export PYTHONPYCACHEPREFIX="$workspace_dir/.local_tools/pycache"
export MPLCONFIGDIR="$workspace_dir/.local_tools/matplotlib"
uv_binary="$workspace_dir/.local_tools/uv/uv"
if [[ ! -x "$uv_binary" ]]; then
  installer="$workspace_dir/.local_tools/downloads/uv-install.sh"
  mkdir -p -- "$(dirname -- "$installer")"
  curl --fail --location --silent --show-error --connect-timeout 20 --max-time 120 \
    --output "$installer" https://astral.sh/uv/install.sh
  # Unmanaged installation also prevents install receipts in the home directory.
  UV_UNMANAGED_INSTALL="$workspace_dir/.local_tools/uv" sh "$installer"
fi
"$uv_binary" --version
"$uv_binary" python install 3.12 --no-bin
if [[ ! -f "$eval_dir/uv.lock" ]]; then
  "$uv_binary" lock --project "$eval_dir" --managed-python
fi
"$uv_binary" sync --project "$eval_dir" --managed-python --locked
echo "Ready. Check: bash tools/map_eval/run.sh python check_env.py"
