# Source this file from Zsh; never source a Zsh setup file from Bash.
if [[ -z "${ZSH_VERSION:-}" ]]; then
  echo "In Bash, source scripts/setenv.bash instead." >&2
  return 2
fi
if [[ ! -r /opt/ros/humble/setup.zsh ]]; then
  echo "[fastlio] ROS Humble is not installed in this terminal's environment." >&2
  echo "[fastlio] If ROS is in Docker: bash docker/run.sh, then source scripts/env.sh inside the container." >&2
  return 2
fi
fastlio_workspace_dir="${${(%):-%x}:A:h:h}"
source /opt/ros/humble/setup.zsh || return
if [[ -f "$fastlio_workspace_dir/install/local_setup.zsh" ]]; then
  source "$fastlio_workspace_dir/install/local_setup.zsh" || return
else
  echo "[fastlio] ROS loaded without workspace overlay. Build before running FAST-LIO; RViz-only viewing does not require a workspace build." >&2
fi
source "$fastlio_workspace_dir/scripts/lib/dds_env.sh" || return
unset fastlio_workspace_dir
