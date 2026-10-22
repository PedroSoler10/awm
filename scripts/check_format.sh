#!/usr/bin/env bash
# Check C++ formatting and style without changing files.
# Usage: scripts/check_format.sh   (use --fix to rewrite files)
set -euo pipefail
cd "$(dirname "$0")/.."
# Every tracked C++ file under src/, include/ and test/, except the nodes not cleaned up yet. Each
# one leaves this list in the refactor task that rewrites it (docs/refactor/):
#   src/nodes/points_identification.cpp  Task 4
#   src/nodes/brainT.cpp                 Task 5
files=$(git ls-files "src/*.cpp" "src/*.h" "include/*.h" "test/*.cpp" "test/*.h" \
  ":!src/nodes/points_identification.cpp" \
  ":!src/nodes/brainT.cpp")
if [[ "${1:-}" == "--fix" ]]; then
  clang-format -i $files
  exit 0
fi
clang-format --dry-run --Werror $files
cpplint --quiet $files
