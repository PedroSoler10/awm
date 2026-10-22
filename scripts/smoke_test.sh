#!/usr/bin/env bash
# Headless simulation smoke test inside the existing VS Code dev container.
#
# Starts awm.launch without the Gazebo window and RViz, waits until the brain has finished
# MIN_CYCLES measurements (or TIMEOUT_S seconds pass), prints a summary, and stops the simulation.
# Build first with scripts/container_build.sh.
#
# Usage, from the host:
#   scripts/smoke_test.sh [roslaunch args for awm.launch]
# Examples:
#   scripts/smoke_test.sh
#   scripts/smoke_test.sh mode:=modular
#   EXTRA_LAUNCH=modular.launch scripts/smoke_test.sh brain:=false
# Environment:
#   MIN_CYCLES    measurements needed to pass (default 3)
#   TIMEOUT_S     seconds to wait for them (default 300)
#   EXTRA_LAUNCH  a second launch file of awm, started 20 s after awm.launch
#   KEEP_RUNNING  1 leaves the simulation running for manual checks (stop it with
#                 scripts/smoke_test.sh --stop)
# The full log stays in the container at /tmp/awm_sim.log (and /tmp/awm_extra.log).

set -euo pipefail

MIN_CYCLES="${MIN_CYCLES:-3}"
TIMEOUT_S="${TIMEOUT_S:-300}"
EXTRA_LAUNCH="${EXTRA_LAUNCH:-}"
KEEP_RUNNING="${KEEP_RUNNING:-0}"

PACKAGE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER=$(docker ps -aq --filter "label=devcontainer.local_folder=$PACKAGE_DIR" | head -1)
if [ -z "$CONTAINER" ]; then
  echo "No dev container for $PACKAGE_DIR. Open the folder in VS Code and run" >&2
  echo "'Dev Containers: Reopen in Container' (it reuses the cached image)." >&2
  exit 1
fi
docker start "$CONTAINER" > /dev/null

in_container() { docker exec "$CONTAINER" bash -c "source /root/shared_ros_ws/devel/setup.bash; $1"; }

# The brackets keep pkill from matching the command line of the shell that runs it.
stop_sim() {
  in_container 'pkill -INT -f "[r]oslaunch"; sleep 15
                pkill -9 -f "[g]zserver|[r]osmaster|[r]oslaunch" || true' || true
}

if [ "${1:-}" == "--stop" ]; then
  stop_sim
  exit 0
fi

if in_container 'pgrep -f "[r]oslaunch|[g]zserver" > /dev/null'; then
  echo "A simulation is already running in the container. Stop it with: $0 --stop" >&2
  exit 1
fi

in_container 'rm -f /tmp/awm_sim.log /tmp/awm_extra.log'
# stdbuf: node output written to a file is block-buffered by default, so progress lines would
# appear minutes late. Line buffering is inherited by the nodes that roslaunch starts.
docker exec -d "$CONTAINER" bash -c "source /root/shared_ros_ws/devel/setup.bash && \
  exec stdbuf -oL -eL roslaunch awm awm.launch gui:=false rviz:=false $* > /tmp/awm_sim.log 2>&1"
if [ -n "$EXTRA_LAUNCH" ]; then
  sleep 20
  docker exec -d "$CONTAINER" bash -c "source /root/shared_ros_ws/devel/setup.bash && \
    exec stdbuf -oL -eL roslaunch awm $EXTRA_LAUNCH > /tmp/awm_extra.log 2>&1"
fi

LOGS='/tmp/awm_sim.log /tmp/awm_extra.log'
count_cycles() { in_container "cat $LOGS 2>/dev/null | grep -c 'MEASURE DONE'" || true; }

echo "Waiting up to ${TIMEOUT_S} s for ${MIN_CYCLES} measurements..."
START=$SECONDS
CYCLES=0
while [ $((SECONDS - START)) -lt "$TIMEOUT_S" ]; do
  CYCLES=$(count_cycles)
  [ "${CYCLES:-0}" -ge "$MIN_CYCLES" ] && break
  if ! in_container 'pgrep -f "[r]oslaunch" > /dev/null'; then
    echo "roslaunch exited early." >&2
    break
  fi
  sleep 10
done

echo "== Nodes"
in_container 'rosnode list' || true
echo "== Brain events"
in_container "cat $LOGS 2>/dev/null | grep -E 'NEW GOAL|MEASURE DONE|GOAL (NOT )?REACHED|EVERYTHING' \
  | sed -E 's/\x1b\[[0-9;]*m//g'" || true
echo "== Errors (counted)"
in_container "cat $LOGS 2>/dev/null | grep -E 'ERROR|FATAL|terminate|process has died' \
  | sed -E 's/\x1b\[[0-9;]*m//g; s/\[[0-9.]+(, [0-9.]+)?\]//' | sort | uniq -c | sort -rn" || true

if [ "$KEEP_RUNNING" == "1" ]; then
  echo "Simulation left running. Stop it with: $0 --stop"
else
  stop_sim
fi

if [ "${CYCLES:-0}" -ge "$MIN_CYCLES" ]; then
  echo "PASS: ${CYCLES} measurements in $((SECONDS - START)) s"
else
  echo "FAIL: ${CYCLES:-0} of ${MIN_CYCLES} measurements in $((SECONDS - START)) s" >&2
  exit 1
fi
