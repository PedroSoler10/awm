#!/usr/bin/env bash
# Runs the brain node under gdb in this terminal.
#
# Usage:
#   scripts/debug_brain.sh [function ...]
#
# Each argument is a function to stop at (default: Brain::control). Examples:
#   scripts/debug_brain.sh
#   scripts/debug_brain.sh Brain::sendGoal Brain::measure awm::findNearestCell
#
# Before it, in other terminals:
#   1. catkin_make -DCMAKE_BUILD_TYPE=Debug      (once, so gdb can show variables)
#   2. roslaunch awm awm.launch brain:=false     (Gazebo, SLAM and navigation without brain)
#   3. rosservice call /gazebo/pause_physics     (optional: freezes the world while you look)
# See "Debugging with gdb" in the README for the gdb commands.

set -euo pipefail

source /opt/ros/noetic/setup.bash
WORKSPACE="$(cd "$(dirname "$0")/../../.." && pwd)"
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash"
fi

BINARY="$WORKSPACE/devel/lib/awm/brain"
if [ ! -x "$BINARY" ]; then
  echo "Not found: $BINARY. Build first: catkin_make -DCMAKE_BUILD_TYPE=Debug" >&2
  exit 1
fi
if ! readelf -S "$BINARY" | grep -q '\.debug_info'; then
  echo "Warning: no debug symbols, variables will show <optimized out>." >&2
  echo "Rebuild with: catkin_make -DCMAKE_BUILD_TYPE=Debug" >&2
fi
if ! timeout 5 rosnode list > /dev/null 2>&1; then
  echo "Warning: no ROS master found. Start 'roslaunch awm awm.launch brain:=false' first." >&2
fi

BREAKPOINTS=("$@")
if [ ${#BREAKPOINTS[@]} -eq 0 ]; then
  BREAKPOINTS=(Brain::control)
fi

GDB_ARGS=(-q -ex "set breakpoint pending on" -ex "set print pretty on")
for function_name in "${BREAKPOINTS[@]}"; do
  GDB_ARGS+=(-ex "break $function_name")
done
GDB_ARGS+=(-ex run)

exec gdb "${GDB_ARGS[@]}" --args "$BINARY"
