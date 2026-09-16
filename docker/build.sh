#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker build --pull -f "$workspace_dir/docker/Dockerfile" \
  --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
  --build-arg INSTALL_RVIZ="${FASTLIO_INSTALL_RVIZ:-0}" \
  -t "${FASTLIO_IMAGE:-fastlio-mid360:humble}" "$@" "$workspace_dir"
