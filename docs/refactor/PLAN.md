# awm refactor plan: clean up, shared library, two modes (standalone `brain` / modular `brainT`)

One session per task, in order. This file has the shared context and rules; each task is in `task-N.md`; `PROGRESS.md` is the log; `RUNNING.md` says how the user runs the sessions.

## Context

`awm` is a ROS Noetic package (TurtleBot3 + gmapping + move_base). Only `src/brain.cpp` is in good shape: it was recently reworked (committed in Step 0 on branch `code-review`) and works. It reads the robot pose from TF, builds a "measure map", picks the closest unmeasured cell, sends it to move_base, waits, "measures" (writes an empty file) and repeats.

The other `.cpp` files are 2021 prototypes. They hold ideas that `brain` lacks: a **safe map** with a caution band around obstacles, **frontier detection** (finding where the map ends, so the robot can explore), and a **goal-points service** with EXPLORATION / MEASUREMENT / FUSION modes. But they are duplicated, never launched, and full of bugs (some crash at once).

Goal: delete what has no value, clean and fix what does, move shared code into a small library with unit tests, get both pipelines working on their own, then add one switch that picks the mode.

### What each file is today (and the decision)

| File | What it does | Main problems | Decision |
|---|---|---|---|
| `src/brain.cpp` | Standalone exploration/measurement state machine (described above). Publishes `/scan_pose`, `/measure_map`, `/move_base_simple/goal`. | Clean. Constants are hard-coded, measurement files go to the cwd, `fn` prefix on method names. | **Keep.** It is the reference. Its behaviour must not change. |
| `src/brainT.cpp` | Older brain. Gets the pose from `/scan_pose` and listens to `/map`, `/safe_map`, `/task_map`. Its goal search is the ancestor of brain's. | All 3 map subscriptions use `mapCallback`, so `map` gets overwritten by the safe/task maps. `statusCallback` reads `status_list[1]`/`[2]` without a size check (undefined behaviour). It publishes AND subscribes to `/task_map`. `tf2_` is declared before `buffer_` (wrong init order). The loop runs inside the constructor. Hard-coded +2 h timezone. `scanf` debug pause. Dead mode 1 test, dead local_map code. Bounds check uses `>` instead of `>=`, and x/y are swapped in the "nothing found" test. | **Keep the idea, rewrite** as the modular brain (Task 5). Merged into `brain` in Task 6. |
| `src/mapper.cpp` | Publishes `/scan_pose`, `/safe_map` (discretise + inflate) and its own `/measure_map`. Writes `.txt` dumps on every map update. | Copy of map_processing. Its measure map duplicates brain's and has a condition that can never be true (`== -1 && > 90`). Same TF init-order bug. Inflation bounds use `> 0`. Maps are passed by value. | **Delete** (Task 1). Its only unique part is a buggy duplicate of brain's measure map. |
| `src/mapperT.cpp` | Byte-for-byte the same as mapper.cpp, plus a commented-out block at the end. | — | **Delete** (Task 1). |
| `src/map_processing.cpp` | Same as mapper, but its measure/task map part is commented out. Publishes `/scan_pose` and `/safe_map`. | Same bugs as mapper. | **Keep, clean up** (Task 3): it becomes the only `/safe_map` producer. |
| `src/points_identification.cpp` | Frontier detection on `/map`. A `GoalPoints` service is planned but commented out. | Variable-length arrays the size of the map on the stack (stack overflow on large maps). Writes to pointer parameters, so results never leave the function. `writeGoals` reads uninitialized pointers (crash). The neighbour loop checks 2x2 cells instead of 3x3. `ROS_INFO` for every frontier cell. | **Keep, rewrite** as the `/goal_points` service (Task 4). |
| `src/measure_server.cpp` | The ROS "add two ints" tutorial renamed. Needs `awm/Measure.h`, which does not exist. | Not built. | **Delete** (Task 1). |
| `src/service_core.cpp` | Goal → measure sequencing skeleton. | Does not compile: `MeasureResult` does not exist, `fnMesure`/`fnNewGoal` are undefined, it publishes `std_msgs::String` on the goal topic. brain already does this sequencing. | **Delete** (Task 1). |
| `src/myTest.cpp` | TF test that prints the base_scan pose in the map frame. | `while(ros::ok)` tests a function pointer, so it is always true. brain's `fnReadScanPose` replaces it. | **Delete** (Task 1). |
| `msg/Num.msg` | Placeholder `int64 num`. Unused. | — | **Delete** (Task 1). |
| `srv/GoalPoints.srv` | Placeholder request (`Mode`, `Num`), response `OtroNum`. | Response is a placeholder. | **Keep, redesign** (Task 4). |

## Target structure

```
include/awm/            # library headers (namespace awm)
  grid.h                # worldToGrid, gridToWorld, cell(), markDisk, inflate, findNearestCells (ring search)
  robot_pose.h          # RobotPose: TF lookup of base_scan in "map" (from brain's fnReadScanPose)
  measure_map.h         # updateMeasureMap, writeMeasurementFile (from brain)
  safe_map.h            # makeSafeMap: discretise + caution band (from map_processing)
  frontiers.h           # frontier cells, clustering, goal per cluster (from points_identification)
  map_dump.h            # optional text dump of a grid (from mapper's fnWriteMap)
src/lib/*.cpp           # library sources -> add_library(awm_core)
src/nodes/brain.cpp     # one executable per file in src/nodes/
src/nodes/map_processing.cpp
src/nodes/points_identification.cpp
src/nodes/brainT.cpp    # exists only from Task 5 to Task 6
test/test_*.cpp         # catkin_add_gtest, pure functions on nav_msgs::OccupancyGrid (no roscore needed)
srv/GoalPoints.srv
launch/awm.launch
```

Data flow in modular mode: `/map` → `map_processing` → `/safe_map`. Then `points_identification` (reads `/map`, `/safe_map`, `/measure_map` and the pose from TF) serves `/goal_points`. `brainT` calls `/goal_points`, publishes `/measure_map`, `/scan_pose` and goals to move_base.

**Final mode switch (Task 6).** brainT is merged into one `brain` executable with a Strategy pattern: an interface `GoalSource` with two implementations.
- `NearestUnmeasuredGoalSource`: today's in-process search.
- `ServiceGoalSource`: calls `/goal_points`.

The private param `~goal_source` chooses: `internal` (default) or `service`. In `awm.launch`, `mode:=standalone|modular` sets that param and starts `map_processing` + `points_identification` only in modular mode.

## Dev container (checked once in Step 0, then only reused)

The VS Code dev container is already built and running: image `vsc-awm-2774…`, built 22 Oct after the latest Dockerfile change. It has gdb, googletest/libgtest-dev, rosunit/rostest, gzserver and catkin_make. It mounts `/home/pedro-soler/shared_ros_ws` at `/root/shared_ros_ws`, on the bridge network, so the ROS master runs inside it. The old image `awm:noetic` (20 Oct, no gdb) is **not used**. clang-format and cpplint are on the host, not in the container.

No task changes the Dockerfile or adds a system dependency (Task 2's `rosunit` test_depend is already installed). So **nobody rebuilds the image or recreates the container**. Code changes only need `catkin_make` inside the container.

Two scripts in `scripts/` (added in Step 0, tested) do all container work from the host. Use them instead of hand-written `docker` commands:

- `scripts/container_build.sh` runs `catkin_make` in the container. `scripts/container_build.sh --test` also runs `catkin_make run_tests_awm` and `catkin_test_results` (useful from Task 2 on, when tests exist).
- `scripts/smoke_test.sh [awm.launch args]` runs the headless sim, waits for `MIN_CYCLES` measurements (default 3, `TIMEOUT_S` default 300), prints nodes, brain events and errors, stops the sim, and prints `PASS` or `FAIL` (exit code 1 on fail). Other options:
  - `EXTRA_LAUNCH=modular.launch` starts a second awm launch file 20 s later.
  - `KEEP_RUNNING=1` leaves the sim running for manual checks (`docker exec "$C" bash -c 'source /root/shared_ros_ws/devel/setup.bash; rostopic echo -n1 /safe_map/info'`). Then stop it with `scripts/smoke_test.sh --stop`.
  - Full logs stay in the container at `/tmp/awm_sim.log` and `/tmp/awm_extra.log`.

Both scripts find the container by its VS Code label (its name is random) and `docker start` it if needed. VS Code stops the container when its window closes; that is harmless. For manual commands:
```bash
C=$(docker ps -aq --filter label=devcontainer.local_folder=/home/pedro-soler/shared_ros_ws/src/awm | head -1)
```
If no container exists, stop and ask the user to open the folder in VS Code and run "Dev Containers: Reopen in Container" (this reuses the cached image). Never `docker build`, `docker run`, `docker rm` or "Rebuild Container".

Lessons from Step 0, for anyone writing their own commands:
- `pkill -f roslaunch` run through `bash -c` also kills that shell. Use `pkill -f "[r]oslaunch"`.
- Node output redirected to a file is block-buffered. Start roslaunch with `stdbuf -oL -eL`, or progress lines arrive minutes late.

## Rules for every session (read before starting a task)

- Do **only your task**. Tasks run in order. Each one starts on branch `code-review`, with the previous task's commit already pushed.
- **Style:** follow `docs/CONFIGURATION.md` and `.clang-format` (ROS C++ style: 2 spaces, 100 columns, `lowerCamelCase` methods without the `fn` prefix, members with a trailing `_`, constants `kName`, `.h` includes first, then C++ standard headers). The brain.cpp at the end of Task 2 is the reference for later tasks. C++14 (Noetic), so no `std::optional`: return `bool` and write results through out-pointers, as brain does today. Pass maps by `const&`. No `using namespace std`, no `#define PI` (use `M_PI`), no `scanf`, no commented-out code. Comments explain *why*.
- **Node pattern:** the constructor only sets things up. `main` calls `ros::spin()`, or `run()` when there is a control loop. Constants become private params (`ros::NodeHandle pnh("~")`) whose defaults are today's values. Publishers of maps are latched.
- **Behaviour of standalone `brain` must not change** (same topics, same goal choice, same measure map), except for changes a task names explicitly.
- **Build and test:** `scripts/container_build.sh` (from Task 2 on: `scripts/container_build.sh --test`). Style check on the host: `scripts/check_format.sh`.
- **Headless sim smoke test:** `scripts/smoke_test.sh [awm.launch args]`. A pass means `PASS` printed, at least 3 `NEW GOAL` → `GOAL REACHED` → `MEASURE DONE` cycles, and no crash, `process has died` or `terminate`. Compare with the baseline in `docs/refactor/PROGRESS.md` (Step 0). If Gazebo cannot start headless, say so in the report and rely on build + unit tests.
- **Finish:**
  - update README sections that your change makes wrong
  - run the build, tests and style check
  - append your section to `docs/refactor/PROGRESS.md`: what changed, what was verified (pass / fail / not run), deviations, open issues for the next task
  - make **one commit** with a clear message (attribution trailer as usual), then `git push origin code-review`
  - report what was verified and what was not

## Verification (end to end)

- `catkin_make run_tests_awm` passes in the dev container (tests for grid, measure_map, safe_map, frontiers and the goal source).
- `scripts/check_format.sh` is clean.
- Sim smoke test: standalone gives ≥3 measure cycles, same as the Step 0 baseline. Modular gives ≥3 cycles and frontier goals that grow `/map`.
- `rosservice call /goal_points` answers in all 3 modes. `rostopic echo -n1 /safe_map/info` matches `/map/info`.
- `git status -sb` after every step: clean tree, nothing ahead of `origin/code-review`.
