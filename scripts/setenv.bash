# Source this file from Bash; paths follow the extracted workspace.
if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "In Zsh, source scripts/setenv.zsh instead." >&2
  return 2
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "[fastlio] ROS Humble is not installed in this terminal's environment." >&2
  echo "[fastlio] If ROS is in Docker: bash docker/run.sh, then source scripts/env.sh inside the container." >&2
  return 2
fi
fastlio_workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash || return
if [[ -f "$fastlio_workspace_dir/install/local_setup.bash" ]]; then
  source "$fastlio_workspace_dir/install/local_setup.bash" || return
else
  echo "[fastlio] ROS loaded without workspace overlay. Build before running FAST-LIO; RViz-only viewing does not require a workspace build." >&2
fi
source "$fastlio_workspace_dir/scripts/lib/dds_env.sh" || return
unset fastlio_workspace_dir
