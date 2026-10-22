#!/usr/bin/env bash
# Builds the workspace inside the existing VS Code dev container (the image is never rebuilt).
#
# Usage, from the host:
#   scripts/container_build.sh          catkin_make
#   scripts/container_build.sh --test   catkin_make, then the unit tests of awm
#
# The container is found by the label VS Code puts on it, because its name is random. If it is
# stopped (VS Code stops it when its window closes) it is started again.

set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER=$(docker ps -aq --filter "label=devcontainer.local_folder=$PACKAGE_DIR" | head -1)
if [ -z "$CONTAINER" ]; then
  echo "No dev container for $PACKAGE_DIR. Open the folder in VS Code and run" >&2
  echo "'Dev Containers: Reopen in Container' (it reuses the cached image)." >&2
  exit 1
fi
docker start "$CONTAINER" > /dev/null

COMMAND='cd /root/shared_ros_ws && catkin_make'
if [ "${1:-}" == "--test" ]; then
  COMMAND+=' && catkin_make run_tests_awm && catkin_test_results build/test_results/awm'
fi
docker exec "$CONTAINER" bash -c "$COMMAND"
