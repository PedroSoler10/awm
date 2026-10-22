# Task 2: Shared library, package layout, brain on top of it

1. Create the layout: `git mv src/brain.cpp src/nodes/brain.cpp`, and the same for the other 3 nodes (their contents do not change in this task). Create `include/awm/` and `src/lib/`.
2. Move brain's static helpers into the library (namespace `awm`) without changing their logic:
   - `grid.h/.cpp`:
     - `worldToGrid` (from `fnWorldToGrid`)
     - `gridToWorld` (the inverse, used in `fnPubGoal`)
     - `cell` (const and non-const versions, from `fnCell`)
     - `markDisk(grid, cx, cy, radius_m, value, keep_value)` (from `fnMarkMeasured`)
     - `inflate(source, threshold, radius_cells, &target, keep_value)` (step 3 of `fnUpdateMeasureMap`)
     - `findNearestCell(grid, x0, y0, wanted_value, max_radius_cells, &x, &y, &angle)` (the ring search in `fnPubGoal`, same 0→359° sweep order)
   - `robot_pose.h/.cpp`: class `RobotPose`. It owns the `tf2_ros::Buffer` and `TransformListener` in the correct declaration order, and has `bool lookup(geometry_msgs::PoseStamped* out)`. Logic from `fnReadScanPose`, stamp 0.
   - `measure_map.h/.cpp`: `updateMeasureMap(map, robot_x, robot_y, params, &measure_map)` (body of `fnUpdateMeasureMap` without the publish), and `writeMeasurementFile(dir, pose)` (file part of `fnMeasure`).
3. Rewrite `src/nodes/brain.cpp` to use the library: drop the `fn` prefix, and make the 5 constants private params with the same defaults. Add `~output_dir` (default `.`) for the measurement files. Keep log texts and topics the same.
4. `CMakeLists.txt`: `set(CMAKE_CXX_STANDARD 14)`, `include_directories(include ...)`, `add_library(awm_core src/lib/*.cpp)` listed explicitly. Every node is `add_executable(<n> src/nodes/<n>.cpp)` linked to `awm_core`. Add `catkin_package(INCLUDE_DIRS include LIBRARIES awm_core ...)`, and install the library and headers.
5. Tests: `test/test_grid.cpp` and `test/test_measure_map.cpp` with `catkin_add_gtest` (add `<test_depend>rosunit</test_depend>`; it is already installed in the container). Cases:
   - worldToGrid/gridToWorld round trip
   - `cell` out of bounds returns nullptr
   - markDisk radius
   - inflate keeps measured cells
   - findNearestCell returns the closest cell, and false when there is none
   - updateMeasureMap after a resize
6. `scripts/check_format.sh`: check every tracked file under `src/ include/ test/`. Run `--fix` only on brain and the library. The other nodes stay as they are until their own task: exclude them with a comment that names the task.
7. `scripts/debug_brain.sh` and the README gdb section: change the default breakpoint `Brain::control`, and any renamed methods.

Done when: build and tests pass, the sim smoke test matches the baseline, and the style check is clean for brain, the library and the tests.
