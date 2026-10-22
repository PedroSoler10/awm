# Progress log

Each session appends one section here before it commits: what changed, what was verified (pass / fail / not run), deviations from the task file, and open issues for the next task.

## Step 0: Baseline (2026-10-22, planning session, Opus 5.5)

**Container.** The VS Code dev container (label `devcontainer.local_folder=/home/pedro-soler/shared_ros_ws/src/awm`, image `vsc-awm-…`, built 2026-10-22) has catkin_make, gzserver, gdb, googletest/libgtest-dev and rosunit/rostest. It stopped once during the session (VS Code sends SIGTERM when its window closes); `docker start` brought it back with no rebuild. clang-format and cpplint run on the host.

**Build.** `catkin_make` builds all 6 current executables (brain, brainT, mapper, mapperT, map_processing, points_identification) with no errors.

**Baseline smoke test** (`scripts/smoke_test.sh`, default args, standalone brain):
- Nodes: `/amcl /brain /gazebo /move_base /robot_state_publisher /rosout /turtlebot3_slam_gmapping`.
- The first measurement happens about 4 s of sim time after start, at the spawn point (-2.06, -0.50). Then every cycle is `NEW GOAL` → `GOAL REACHED (status 3)` → `MEASURE DONE` after 3 s, with goals about 0.5 m apart.
- First goals: cell (169,191) angle 3, then (168,200) angle 88, then (160,206) angle 139, then (170,211) angle 27.
- In one longer run: 12 measurements in about 130 s of sim time, all goals reached, no `NOT REACHED`, no errors. `MIN_CYCLES=3` passes in about 50 s of wall time.
- Known warnings, present before any change (not regressions):
  - `Request for map failed; trying again...` (hundreds; amcl asks for a static map, but gmapping provides `/map`)
  - `DWA planner failed to produce path` (a few)
  - `Parameter "plugins" not provided, loading pre-Hydro parameters` (once per costmap)
- Measurement files are written to `/root/.ros` inside the container (roslaunch's working directory).

**Tooling added.**
- `scripts/container_build.sh [--test]`: builds (and tests) in the dev container.
- `scripts/smoke_test.sh`: headless sim test with PASS/FAIL. It uses `stdbuf -oL` because node output to a file is block-buffered, and `pkill -f "[r]oslaunch"` so it does not kill its own shell.

## Task 1: Remove files with no value (2026-10-22)

**Changed.** `git rm` of `src/mapper.cpp`, `src/mapperT.cpp`, `src/myTest.cpp`, `src/measure_server.cpp`, `src/service_core.cpp`, `msg/Num.msg`. `CMakeLists.txt`: removed `mapper`/`mapperT` from `AWM_EXECUTABLES`, `add_message_files`, and the "Not built on purpose" block. README contents list fixed (no `mapper`, no `msg/`, `Num.msg`). Comments in `CMakeLists.txt` and `package.xml` that mentioned `msg/` now say `srv/` only.

**Verified.**
- `scripts/container_build.sh`: pass; 4 executables (brain, brainT, map_processing, points_identification). Stale `mapper`/`mapperT` binaries from the earlier build were deleted from `devel/lib/awm` by hand (untracked build output).
- `scripts/smoke_test.sh`: PASS, 3 measurements in 48 s, same node list as the baseline, no errors. First goal (169,191) angle 3 matches the baseline; later goals differ by a few cells (e.g. (168,201) vs (168,200)), which is gmapping map variation, not a code change.
- `scripts/check_format.sh`: clean.

**Deviations.** None. `package.xml` keeps `message_generation`, still needed for `GoalPoints.srv`.

**Open for Task 2.** The empty `msg/` directory is gone; nothing else references it.
