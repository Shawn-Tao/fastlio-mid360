#!/usr/bin/env bash
# Persistent CPU-only container; existing containers are validated, not replaced.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
container_id="${FASTLIO_CONTAINER:-fastlio-mid360}"
container_workspace=/workspace/fastlio-mid360_space
if [[ $# -gt 1 || ( $# == 1 && "$1" != --detach ) ]]; then
  echo "Usage: bash docker/run.sh [--detach]" >&2; exit 2
fi
if docker inspect "$container_id" >/dev/null 2>&1; then
  existing_source="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/workspace/fastlio-mid360_space"}}{{.Source}}{{end}}{{end}}' "$container_id")"
  existing_network="$(docker inspect --format '{{.HostConfig.NetworkMode}}' "$container_id")"
  if [[ "$existing_source" != "$workspace_dir" || "$existing_network" != host ]]; then
    echo "Existing container has a different workspace/network. Choose a new FASTLIO_CONTAINER name; nothing was replaced." >&2; exit 2
  fi
  if [[ "$(docker inspect --format '{{.State.Running}}' "$container_id")" != true ]]; then docker start "$container_id"; fi
else
  gui_options=()
  if [[ "${FASTLIO_DOCKER_GUI:-0}" == 1 ]]; then
    if [[ ! -d /tmp/.X11-unix || -z "${DISPLAY:-}" ]]; then
      echo "GUI requested but DISPLAY/X11 socket is absent. Use PC RViz or headless mode." >&2; exit 2
    fi
    gui_options=(--mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly -e DISPLAY -e QT_X11_NO_MITSHM=1)
  fi
  docker run -d --name "$container_id" --network host \
    --mount "type=bind,src=$workspace_dir,dst=$container_workspace" \
    -w "$container_workspace" "${gui_options[@]}" \
    "${FASTLIO_IMAGE:-fastlio-mid360:humble}" sleep infinity
fi
if [[ "${1:-}" == --detach ]]; then
  echo "Container ready: $container_id. Run bash scripts/build.sh from this workspace."
  exit 0
fi
terminal=(-i)
if [[ -t 0 && -t 1 ]]; then terminal=(-it); fi
exec docker exec "${terminal[@]}" "$container_id" bash
