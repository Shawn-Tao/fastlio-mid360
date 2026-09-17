#!/usr/bin/env bash
# Archived alias; prefer scripts/setenv.bash or scripts/setenv.zsh.
if [[ -n "${ZSH_VERSION:-}" ]]; then
  source "${${(%):-%x}:A:h:h}/setenv.zsh"
else
  source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/setenv.bash"
fi
