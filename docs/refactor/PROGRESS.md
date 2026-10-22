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

## Task 4: points_identification → /goal_points service (2026-10-22)

**Changed.**
- `srv/GoalPoints.srv` as in the task (constants `EXPLORATION`/`MEASUREMENT`/`FUSION`, `mode`, `num`; response `geometry_msgs/Point[] points`). `generate_messages(DEPENDENCIES geometry_msgs)`.
- `grid.h/.cpp`: new `struct GridCell {x, y}` and `findNearestCells(grid, x0, y0, wanted, max_radius, n, &out, angles_deg = nullptr)` (n = 0 collects all, no duplicate cells, closest first). `findNearestCell` is now a call with n = 1, same result and angle; its 5 tests pass unchanged.
- `frontiers.h/.cpp`: `findFrontierCells` (map 0, safe_map 0, any of the 8 neighbours unknown; false if the maps differ in size), `clusterFrontiers` (BFS, 8-connectivity, filters by `min_cluster_size`, hash-set of cell keys, no VLAs), `frontierGoals` (cell nearest each centroid, stable sort by distance to the robot). All in grid cells.
- `src/nodes/points_identification.cpp` rewritten: `/map`, `/safe_map`, `/measure_map` kept as shared `ConstPtr`; pose from `awm::RobotPose` at request time; `/goal_points`. Bad mode → `ROS_WARN` + `false`. Missing maps/pose → empty list + `ROS_WARN_THROTTLE`. FUSION treats a missing `/measure_map` as "nothing to measure". Params `~min_cluster_size` 5, `~search_limit_cells` 200, `~dump_maps` false, `~dump_dir` ".". The old file (prototype code, `fn*` helpers, dead task-map code) is gone.
- `map_dump`: second overload `writeMapText(grid, path, marks)` draws `8` over the marked cells; used by `~dump_maps`. The first overload is unchanged.
- `test/test_frontiers.cpp` (11 tests: boundary = one cluster, small cluster filtered, goals sorted, 3x3 diagonal check, caution/obstacle cells excluded, size mismatch, no aliasing across the grid edge, centroid goal is a real cell), 3 new tests in `test_grid.cpp`, 1 in `test_safe_map.cpp`.
- `scripts/check_format.sh`: points_identification removed from the exclusions. README: contents list and a "Goal points service" section.

**Verified.**
- `scripts/container_build.sh --test`: pass, 78 entries, 0 errors, 0 failures.
- `scripts/check_format.sh`: clean.
- `KEEP_RUNNING=1 scripts/smoke_test.sh`: PASS, 3 measurements in 32 s, first goal (169,191) angle 3 as in the baseline. Then `map_processing` and `points_identification _dump_maps:=true` via `docker exec -d`: `rosservice call /goal_points "{mode: 1, num: 5}"` returned 5 points inside the map; `num: 0` gave 14 (one per cluster). Mode 2 gave `[]` with the warning "NO /measure_map YET" until brain published its next measure map, then returned 5 points (2193 with `num: 0`, bounded by `~search_limit_cells`); mode 3 gave exploration goals before that and measurement goals after. Mode 9 → `ERROR: service ... responded with an error` and the warning. `/safe_map/info` equals `/map/info`. `map_with_goals.txt` was written with 8s. Sim stopped afterwards.
- Not run: `~min_cluster_size` / `~search_limit_cells` other than the defaults; a real map large enough to time the service call.

**Deviations.**
- `findNearestCells` has an optional `angles_deg` out-param (the task lists none): `findNearestCell` must still return its angle. It returns `bool` (found any).
- The `map_dump` overload with marks was not in the task; `~dump_maps` needs it. The old node wrote the symbol and an extra `8`; now the 8 replaces the symbol.
- `getGoalPoints` has `// NOLINT(runtime/references)`: `advertiseService` requires non-const references.
- Frontier "free" is `map == 0` as the task says. gmapping only publishes 0/100/-1, but a map with other low values (e.g. 10) would give no frontiers.

**Open for Task 5/6.**
- ~~`brain` publishes `/measure_map` unlatched~~: fixed right after Task 4, see "Follow-up" below.
- MEASUREMENT returns the nearest cells, which are mostly neighbours of each other (5 goals within about 0.4 m). The caller should use the first one, or ask for more and filter.
- The ring sweep samples 360 angles, so with `num: 0` it can skip cells at distances over about 57 cells.
- `awm.launch` does not start `map_processing` or `points_identification` yet (Task 6).

### Follow-up after Task 4: latched /measure_map (2026-10-22)

`brain` now advertises `/measure_map` latched (the node-pattern rule; Task 2 had left it unlatched to keep behaviour identical). Topic, queue size and content are unchanged; only a late subscriber (such as `points_identification`) now gets the last map at once. Verified: `scripts/check_format.sh` clean, build ok, `KEEP_RUNNING=1 scripts/smoke_test.sh` PASS (3 measurements in 32 s), and `rostopic echo -n1 /measure_map/header` started after the measurements returned a message immediately.

## Task 5: brainT → modular brain that uses the services (2026-10-22)

**Changed.**
- `src/nodes/brainT.cpp` rewritten on the template of `brain.cpp` (class `BrainT`, node name `brain`). Same `statusCallback`, `RobotPose`, `updateMeasureMap`, `writeMeasurementFile`, `markDisk` after each goal, 10 Hz `run()` loop. It subscribes only to `/safe_map` and `/move_base/status`; publishes `/scan_pose`, latched `/measure_map` and `/move_base_simple/goal`.
- The measure map is built from `/safe_map` with `occupied_threshold = kSafeCaution - 1` (so caution cells, value 50, are blocked and never goals) and `inflate_radius_cells = 0` (the safe map already holds the margin).
- Goal choice: `ros::service::waitForService("/goal_points", 1 s)` with a throttled warning, then a call with `~goal_mode` (default 3 FUSION, validated at start-up: anything else is `ROS_FATAL` + shutdown) and `num: 1`. An empty answer logs `NO MORE GOAL POINTS` and goes to `DONE`. The goal faces the direction from the robot to the point (the service gives positions only). The log line `NEW GOAL: cell x: y: angle:` is kept.
- All the bugs and dead code of the old brainT listed in PLAN.md are gone (it is a new file). Private params: `~goal_mode`, `~measure_duration_s` 3.0, `~measure_radius_m` 0.5, `~output_dir` ".".
- New `launch/modular.launch` (arg `goal_mode`): `map_processing`, `points_identification`, `brainT` as node `brain`. `awm.launch` unchanged. Used as `roslaunch awm awm.launch brain:=false` + `roslaunch awm modular.launch`.
- `scripts/check_format.sh`: no exclusions left. README: contents list, "Modular pipeline (experimental)" section, goal-points section mentions `brainT`.

**Verified.**
- `scripts/container_build.sh --test`: pass, 78 entries, 0 errors, 0 failures. `scripts/check_format.sh`: clean (brainT.cpp included).
- Standalone `scripts/smoke_test.sh`: PASS, 3 measurements in 48 s; first goal (169,191) angle 3 as in the baseline, then (168,201), (160,207) as in Task 2.
- Modular `TIMEOUT_S=600 EXTRA_LAUNCH=modular.launch scripts/smoke_test.sh brain:=false`: PASS, 3 measurements in 47 s; nodes include `/map_processing` and `/points_identification`; first goals (169,191), (168,200), (160,206).
- Long modular run (`KEEP_RUNNING=1`, left running ~25 min of sim time): 69 goals, all reached, no `NOT REACHED`, then `NO MORE GOAL POINTS`. `/map` stayed 384x384 the whole time, known cells grew only from 8836 to 8846; at the end `/goal_points` mode 2 and mode 1 both returned nothing.
- Frontier path through brainT: `EXTRA_LAUNCH="modular.launch goal_mode:=1"` PASS, 3 cycles in 22 s, `GOAL POINTS: mode 1, 1 points` before each goal; at that time mode 1 had 17 clusters.
- Not run: `~goal_mode:=2`; the invalid `~goal_mode` branch; `~output_dir` other than the default.

**Deviations.**
- Extra state `CHOOSING_GOAL` between `MEASURING` and `REACHING_GOAL`. The measurement must be recorded once, while the service call may have to be repeated until `/goal_points` is up.
- **The "done when" item "a frontier goal that grows the map (`/map` width or height increases)" was not reached in the sim, and with this design it is not expected to be.** The LiDAR discovers the space before the robot gets there, so the map is fully discovered before it is fully measured: in the long run the known cells were already 8836 at about 800 s of sim time and only reached 8846, and no frontier cluster was left when the measurement goals ran out. FUSION takes measurement goals first, so it never fell through to EXPLORATION, and the robot ended with `NO MORE GOAL POINTS`. In addition, gmapping is configured with `xmin/xmax/ymin/ymax = ±10`, so `/map` is 384x384 from the start and its size would only change if the robot mapped beyond 20 m, which the enclosed ~6 m TurtleBot3 world does not allow. What was shown instead: frontier goals work through brainT (`goal_mode:=1` run, goals taken from `/goal_points` mode 1), and FUSION chooses measurement goals first. I changed neither the world nor the gmapping size.

**Open for Task 6.**
- Race: `brainT` publishes `/measure_map` and calls `/goal_points` in the same tick; if `points_identification` has not yet received the new map, MEASUREMENT could use the previous one. No bad goal was seen in the runs (all 69 reached, none repeated at the robot position), but nothing guarantees the order.
- FUSION frontiers are not filtered by the measure map, so a frontier that stays a frontier after a visit would be chosen again and again. It did not happen in these runs.
- `/measure_map` is latched by both brains; `brainT` and `brain` use the same node name `brain`, so Task 6's merge can drop `brainT` without changing topics.
