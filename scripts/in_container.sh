#!/usr/bin/env bash
# Select a running container by bind mount, never by a machine-specific ID.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
script_name="${1:?Pass a workspace script name}"
shift
case "$script_name" in
  build.sh|test.sh|run.sh|check_network.sh) ;;
  *) echo "Unsupported container script: $script_name" >&2; exit 2 ;;
esac
find_workspace_mount() {
  local candidate="$1" mount_source mount_destination longest_mount=0 mounts
  mounts="$(docker inspect --format '{{range .Mounts}}{{.Source}}{{"\t"}}{{.Destination}}{{"\n"}}{{end}}' "$candidate")" || return
  matched_workspace=''
  while IFS=$'\t' read -r mount_source mount_destination; do
    if [[ -n "$mount_source" && ( "$workspace_dir" == "$mount_source" || "$workspace_dir" == "$mount_source/"* ) ]]; then
      if [[ ${#mount_source} -gt $longest_mount ]]; then
        matched_workspace="$mount_destination${workspace_dir#"$mount_source"}"
        longest_mount=${#mount_source}
      fi
    fi
  done <<< "$mounts"
}
container_id="${FASTLIO_CONTAINER:-}"
container_workspace="${FASTLIO_CONTAINER_WORKSPACE:-}"
if [[ -z "$container_id" ]]; then
  candidates=()
  running_containers="$(docker ps -q)"
  while IFS= read -r candidate; do
    [[ -n "$candidate" ]] || continue
    find_workspace_mount "$candidate"
    if [[ -n "$matched_workspace" ]]; then candidates+=("$candidate"); fi
  done <<< "$running_containers"
  if [[ ${#candidates[@]} != 1 ]]; then
    echo "Found ${#candidates[@]} running containers mounting this workspace. Set FASTLIO_CONTAINER explicitly, or start one with bash docker/run.sh --detach." >&2
    exit 2
  fi
  container_id="${candidates[0]}"
fi
if [[ -z "$container_workspace" ]]; then
  find_workspace_mount "$container_id"
  container_workspace="$matched_workspace"
fi
if [[ -z "$container_workspace" ]]; then
  echo "Workspace is not mounted in $container_id. Set FASTLIO_CONTAINER_WORKSPACE to its container path." >&2
  exit 2
fi
docker_terminal=(-i)
if [[ -t 0 && -t 1 ]]; then docker_terminal=(-it); fi
docker_environment=(
  -e FASTLIO_NATIVE=1
  -e FASTLIO_BUILD_JOBS="${FASTLIO_BUILD_JOBS:-2}"
  -e FASTLIO_TEST_DOMAIN="${FASTLIO_TEST_DOMAIN:-91}")
# Custom absolute paths must be accessible inside the container.
for variable in FASTLIO_DDS ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI DISPLAY QT_X11_NO_MITSHM LIBGL_ALWAYS_SOFTWARE; do
  if [[ -v "$variable" ]]; then docker_environment+=(-e "$variable=${!variable}"); fi
done
echo "[fastlio] container=$container_id workspace=$container_workspace"
exec docker exec "${docker_terminal[@]}" -w "$container_workspace" \
  "${docker_environment[@]}" "$container_id" \
  bash "$container_workspace/scripts/$script_name" "$@"
