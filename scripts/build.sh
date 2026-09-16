#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/scripts/common.sh"
fastlio_dispatch build.sh "$@"
source /opt/ros/humble/setup.bash
cd "$workspace_dir"
export CMAKE_BUILD_PARALLEL_LEVEL="${FASTLIO_BUILD_JOBS:-2}"
if [[ ! "$CMAKE_BUILD_PARALLEL_LEVEL" =~ ^[1-9][0-9]*$ ]]; then
  echo "FASTLIO_BUILD_JOBS must be a positive integer." >&2
  exit 2
fi
# Humble's colcon explicitly supplies make -j<CPU count> unless MAKEFLAGS
# already contains a job limit; CMAKE_BUILD_PARALLEL_LEVEL alone is not enough.
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}}"
exec colcon build --symlink-install --executor sequential \
  --event-handlers console_direct+ "$@"
