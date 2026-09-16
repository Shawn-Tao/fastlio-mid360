#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/scripts/common.sh"
fastlio_dispatch test.sh "$@"
source /opt/ros/humble/setup.bash
if [[ ! -f "$workspace_dir/install/local_setup.bash" ]]; then
  echo "Build the workspace first: bash scripts/build.sh" >&2
  exit 2
fi
source "$workspace_dir/install/local_setup.bash"
cd "$workspace_dir"
colcon test --event-handlers console_direct+ "$@"
colcon test-result --verbose
# Local graph only; no hardware packets or bag/map writes in the smoke test.
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${FASTLIO_TEST_DOMAIN:-91}"
export ROS_LOG_DIR="$workspace_dir/log/smoke"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
python3 "$workspace_dir/tests/smoke_test.py"
