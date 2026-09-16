# Source this file from Bash; paths follow the extracted workspace.
if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "In Zsh, source scripts/setenv.zsh instead." >&2
  return 2
fi
fastlio_workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash || return
if [[ -f "$fastlio_workspace_dir/install/local_setup.bash" ]]; then
  source "$fastlio_workspace_dir/install/local_setup.bash" || return
else
  echo "[fastlio] ROS loaded; build this workspace before running nodes." >&2
fi
source "$fastlio_workspace_dir/scripts/dds_env.sh" || return
unset fastlio_workspace_dir
