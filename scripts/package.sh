#!/usr/bin/env bash
# Deployment artifact, not a backup: omit machine-specific/generated files.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_parent="$(dirname -- "$workspace_dir")"
workspace_name="$(basename -- "$workspace_dir")"
if [[ $# -gt 1 ]]; then echo "Usage: bash scripts/package.sh [/path/new.zip]" >&2; exit 2; fi
archive_path="${1:-$workspace_parent/${workspace_name}_jetson_$(date +%Y%m%d_%H%M%S).zip}"
archive_path="$(realpath -m -- "$archive_path")"
if [[ "$archive_path" != *.zip || -e "$archive_path" || -e "$archive_path.sha256" ]]; then
  echo "Choose a NEW .zip path; existing artifacts are never overwritten." >&2; exit 2
fi
if [[ "$archive_path" == "$workspace_dir/"* ]]; then
  echo "Put the ZIP outside the workspace to prevent self-inclusion." >&2; exit 2
fi
mkdir -p -- "$(dirname -- "$archive_path")"
cd "$workspace_parent"
zip -q -r -y "$archive_path" "$workspace_name" -x \
  "$workspace_name/build/*" "$workspace_name/install/*" "$workspace_name/log/*" \
  "$workspace_name/bags/*" "$workspace_name/reference/*" "$workspace_name/maps/*" \
  "$workspace_name/records/*" "$workspace_name/Record_Path/*" \
  "$workspace_name/scripts/backup" "$workspace_name/scripts/backup/*" \
  "$workspace_name/config/local/*" "$workspace_name/secrets/*" \
  '*/.git' '*/.git/*' '*/.agents/*' '*/.codex/*' '*/__pycache__/*' '*.pyc' '*.pyo' \
  '*/.vscode/*' '*/.idea/*' '*/.pytest_cache/*' '*/.mypy_cache/*' '*/.ruff_cache/*' \
  '*.local.json' '*.local.yaml' '*.local.yml' '*.local.xml' '*.local.sh' '*.local.bash' '*.local.zsh' \
  '*/.env' '*/.env.*' \
  '*.zip' '*.zip.sha256' '*.tmp*' '*_bak.*' '*.swp' '*.swo' '.DS_Store' \
  "$workspace_name/src/FAST_LIO/Log/*.txt" "$workspace_name/src/FAST_LIO/PCD/*"
unzip -tq "$archive_path"
python3 "$workspace_dir/tests/check_archive.py" "$archive_path"
# Basename in the checksum permits sha256sum -c after copying to Jetson.
cd "$(dirname -- "$archive_path")"
sha256sum -- "$(basename -- "$archive_path")" > "$archive_path.sha256"
echo "Package: $archive_path"
echo "Checksum: $archive_path.sha256"
