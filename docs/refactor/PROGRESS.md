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

## Task 2: Shared library, package layout, brain on top of it (2026-10-22)

**Changed.**
- Layout: `git mv` of the 4 nodes to `src/nodes/` (only `brain.cpp` changed content). New `include/awm/` and `src/lib/` (library `awm_core`, namespace `awm`), `test/`.
- `grid.h/.cpp`: `worldToGrid`, `gridToWorld`, `cell` (const and non-const), `markDisk`, `inflate` (sets 100), `findNearestCell` (same d = 0..max, 0→359° sweep; returns the angle in degrees). Logic copied from brain's `fn*` helpers.
- `robot_pose.h/.cpp`: `RobotPose` owns `tf2_ros::Buffer` then `TransformListener` (correct order). `lookup()` is `fnReadScanPose` without the publish: stamp 0, same `ERROR WITH SCAN POSE` warning, stamps the result with `now()`, leaves `*out` unchanged on failure.
- `measure_map.h/.cpp`: `kBlocked`/`kMeasured`/`kToMeasure`, `MeasureMapParams`, `updateMeasureMap` (body of `fnUpdateMeasureMap` without stamp and publish, so it runs without ROS time), `writeMeasurementFile(dir, pose, &file_name)`. The extra `file_name` out-param keeps brain's log line `MEASURE DONE: <name>` exactly as before.
- `src/nodes/brain.cpp`: uses the library, no `fn` prefix (`control`, `startMeasuring`, `measure`, `updateMeasureMap`, `pubGoal`, `readScanPose`, `goalFinished`). Private params with the old defaults: `~measure_duration_s` 3.0, `~measure_radius_m` 0.5, `~inflate_radius_cells` 6, `~search_limit_cells` 200, `~occupied_threshold` 90, plus `~output_dir` ".". Topics, queue sizes and log texts unchanged.
- `CMakeLists.txt`: C++14, `include_directories(include ...)`, `add_library(awm_core ...)` with the 3 sources listed, every node built from `src/nodes/<n>.cpp` and linked to `awm_core`, `catkin_package(INCLUDE_DIRS include LIBRARIES awm_core ...)`, install of library and headers, `catkin_add_gtest` for `test_grid` and `test_measure_map`. `package.xml`: `<test_depend>rosunit</test_depend>`.
- Tests (15): round trip and rounding of world/grid, `cell` out of bounds (both versions), `markDisk` radius / keep value / border clipping, `inflate` keeps measured cells and ignores the threshold value, `findNearestCell` closest / sweep order / distance 0 / none / limit, `updateMeasureMap` first update / keeps measured cells / reset after resize / margin skips measured cells.
- `scripts/check_format.sh`: all tracked C++ under `src/ include/ test/`, excluding `map_processing.cpp` (Task 3), `points_identification.cpp` (Task 4), `brainT.cpp` (Task 5). `CPPLINT.cfg`: `root=include`, so header guards are `AWM_<NAME>_H_`.
- Docs: README contents list, brain parameter table, gdb example names, VS Code breakpoint path; `scripts/debug_brain.sh` example names (default breakpoint was already `Brain::control`); `docs/CONFIGURATION.md` include order (package headers last, as clang-format sorts them) and what the style check covers.

**Verified.**
- `scripts/container_build.sh --test`: pass. All 4 executables and the 2 test binaries build; `catkin_test_results`: 0 errors, 0 failures (it reports 30 entries for the 15 tests).
- `scripts/smoke_test.sh`: PASS, 3 measurements in 48 s. Same node list as the baseline, no `process has died` or `terminate`. Goals: (169,191) angle 3, (168,201) angle 88, (160,207) angle 139: the first equals the baseline, the others match Task 1's run within one cell (gmapping variation). Measurement files still land in `/root/.ros`.
- `scripts/check_format.sh`: clean.
- Not run: `~output_dir` set to another folder (only the default was exercised by the smoke test).

**Deviations.**
- `writeMeasurementFile` has a third out-param `file_name` (the task names only `dir, pose`), needed to keep the log text identical.
- `/measure_map` stays not latched: the node-pattern rule says map publishers are latched, but this task requires brain's behaviour to stay the same and does not name that change. Worth doing in Task 6 or whenever brain's behaviour may change.

**Open for Task 3.**
- `OccupancyGrid::info.resolution` is `float32` (0.05f ≈ 0.0500000007), so a cell exactly at `radius_m` is outside `markDisk` (e.g. 0.1 m = 2 cells excludes the 2-cell ring). Same as the baseline; the tests use 0.11 m. Keep it in mind for safe-map distances.
- `awm_core` links all of `${catkin_LIBRARIES}`; fine for now.

## Task 3: map_processing → clean /safe_map producer (2026-10-22)

**Changed.**
- `safe_map.h/.cpp`: `makeSafeMap(map, occupied_threshold, caution_threshold, inflate_radius_cells, &safe_map)` plus constants `kSafeUnknown/Free/Caution/Occupied`. It copies the map (header and info), discretises (≥ occupied → 100, ≥ caution → 50, ≥ 0 → 0, else -1), then marks free cells in the square around each obstacle as 50. The band is read from the source map, so it never chains and never touches an occupied cell. Bounds use `awm::cell`, so row 0 and column 0 are included (old test was `> 0`).
- `map_dump.h/.cpp`: `writeMapText(grid, path)`, symbols `* + · space` as before. It returns `false` if the file cannot be opened.
- `src/nodes/map_processing.cpp` rewritten: `/map` (const ref) → `/safe_map` (latched); params `~occupied_threshold` 90, `~caution_threshold` 50, `~inflate_radius_cells` 6, `~dump_maps` false, `~dump_dir` "."; dump files deleted at start-up only when dumps are on. No TF, no `/scan_pose`, no loop, no `wait()`, no print helpers, no task-map code.
- `test/test_safe_map.cpp` (9 tests): discretisation and thresholds, header/size kept, band around a 10x10 obstacle, band at the borders, unknown stays unknown, band not chained, empty map, text dump layout and bad path. `CMakeLists.txt`: new sources in `awm_core`, new test. `scripts/check_format.sh`: map_processing removed from the exclusions. README: contents list mentions the safe map and dump helpers.

**Verified.**
- `scripts/container_build.sh --test`: pass, 48 entries, 0 errors, 0 failures.
- `scripts/check_format.sh`: clean.
- `KEEP_RUNNING=1 scripts/smoke_test.sh`: PASS, 3 measurements in 33 s, same nodes and first goal (169,191) as the baseline. Then `rosrun awm map_processing _dump_maps:=true _dump_dir:=/tmp/dump` (via `docker exec -d`) was the only `/safe_map` publisher; `rostopic echo -n1 /safe_map/info` equals `/map/info` (384x384, 0.05, origin -10,-10); values were only -1/0/50/100; `map.txt` and `safe_map.txt` were written. Sim stopped afterwards.
- Not run: other values of the params through a launch file (only defaults and the dump params).

**Deviations.**
- The old code also turned **unknown** cells (-1 < 50) near an obstacle into 50. Now unknown stays -1, because the task lists -1 as unknown and Task 4's frontier detection needs unknown cells next to walls. Say so if the old behaviour was wanted.
- `awm::inflate` was not reused: it writes 100 and the safe map needs 50 on known-free cells only. The loop in `makeSafeMap` uses `awm::cell`.
- `writeMapText` fixes the old index mix-up (`i` over height used with `width`; only right for square maps) and prints the top row (highest y) first, so the text has the orientation of RViz. Symbols are unchanged.
- `awm.launch` does not start `map_processing` yet (not asked in this task).

**Open for Task 4.** `/safe_map` has the same header (frame, stamp of the source map) as `/map`. Free cells are exactly 0; caution is 50; unknown is -1.
