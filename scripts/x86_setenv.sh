#!/usr/bin/env bash
# Compatibility name, source from Bash or Zsh; no x86-only paths.
if [[ -n "${ZSH_VERSION:-}" ]]; then
  source "${${(%):-%x}:A:h}/setenv.zsh"
else
  source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/setenv.bash"
fi
