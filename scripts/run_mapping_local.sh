#!/usr/bin/env bash
# Safe compatibility wrapper: GUI and bag recording stay opt-in.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/run.sh" mapping "$@"
