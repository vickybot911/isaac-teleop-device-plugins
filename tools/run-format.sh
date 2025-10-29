#!/usr/bin/env bash
set -euo pipefail

CLANG_FORMAT_BIN=${1:-clang-format-14}

if ! command -v git >/dev/null 2>&1; then
  echo "git is required to list files" >&2
  exit 1
fi

IGNORE_FILE="${PROJECT_ROOT:-$(pwd)}/.clang-format-ignore"

if [ -f "$IGNORE_FILE" ]; then
  # Build exclusion patterns for git ls-files pathspec magic
  EXCLUDES=()
  while IFS= read -r line; do
    [ -z "$line" ] && continue
    [[ "$line" =~ ^# ]] && continue
    EXCLUDES+=(":(exclude)$line")
  done < "$IGNORE_FILE"
  mapfile -t files < <(git ls-files -- '*.c' '*.cc' '*.cpp' '*.cxx' '*.h' '*.hpp' "${EXCLUDES[@]}" || true)
else
  mapfile -t files < <(git ls-files -- '*.c' '*.cc' '*.cpp' '*.cxx' '*.h' '*.hpp' || true)
fi
if [ ${#files[@]} -eq 0 ]; then
  echo "No C/C++ files to format."
  exit 0
fi

"${CLANG_FORMAT_BIN}" -i "${files[@]}"
echo "Formatted ${#files[@]} file(s)."


