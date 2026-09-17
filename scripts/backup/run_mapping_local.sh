#!/usr/bin/env bash
# Archived compatibility wrapper; prefer scripts/run.sh mapping.
# RViz2 defaults off; pass --rviz (or rviz:=true) to enable. --help lists options.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$script_dir/run.sh" mapping "$@"
