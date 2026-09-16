# Source this file from Zsh; never source a Zsh setup file from Bash.
fastlio_workspace_dir="${${(%):-%x}:A:h:h}"
source /opt/ros/humble/setup.zsh || return
if [[ -f "$fastlio_workspace_dir/install/local_setup.zsh" ]]; then
  source "$fastlio_workspace_dir/install/local_setup.zsh" || return
else
  echo "[fastlio] ROS loaded; build this workspace before running nodes." >&2
fi
source "$fastlio_workspace_dir/scripts/dds_env.sh" || return
unset fastlio_workspace_dir
