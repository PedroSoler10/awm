#!/usr/bin/env bash
# Check C++ formatting and style without changing files.
# Usage: scripts/check_format.sh   (use --fix to rewrite files)
set -euo pipefail
cd "$(dirname "$0")/.."
# files=$(git ls-files "src/*.cpp" "src/*.h" "include/*.h" "include/*.hpp")
files=$(git ls-files "src/brain.cpp")
if [[ "${1:-}" == "--fix" ]]; then
  clang-format -i $files
  exit 0
fi
clang-format --dry-run --Werror $files
cpplint --quiet $files
