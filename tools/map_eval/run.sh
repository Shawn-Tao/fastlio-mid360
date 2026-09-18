#!/usr/bin/env bash
# A child process only: never source/activate this environment into a ROS shell.
set -euo pipefail
eval_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$eval_dir/../.." && pwd)"
if [[ $# -eq 0 || "${1:-}" == --help ]]; then
  echo "Usage: bash tools/map_eval/run.sh COMMAND [ARG ...]"
  echo "Example: bash tools/map_eval/run.sh python check_env.py --maps ../../../map_bak"
  exit 0
fi
uv_binary="$workspace_dir/.local_tools/uv/uv"
if [[ ! -x "$uv_binary" || ! -f "$eval_dir/uv.lock" ]]; then
  echo "Set up first: bash tools/map_eval/setup.sh" >&2
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
exec "$uv_binary" --directory "$eval_dir" run --managed-python --locked "$@"
