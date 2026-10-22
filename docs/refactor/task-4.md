# Task 4: points_identification → `/goal_points` service

1. Redesign `srv/GoalPoints.srv`:
   ```
   uint8 EXPLORATION=1   # frontier goals: free cells at the edge of the known map
   uint8 MEASUREMENT=2   # nearest unmeasured cells of /measure_map
   uint8 FUSION=3        # MEASUREMENT first; if empty, EXPLORATION
   uint8 mode
   uint8 num             # max goals returned, 0 = all
   ---
   geometry_msgs/Point[] points   # "map" frame, best first
   ```
   Then `generate_messages(DEPENDENCIES geometry_msgs)`.
2. Library `frontiers.h/.cpp`:
   - `findFrontierCells(map, safe_map)`: free cells (map 0) with at least one unknown neighbour among the 8 (a correct 3x3 check), that are 0 in safe_map. Result in `std::vector`, never stack VLAs.
   - `clusterFrontiers(cells, width, min_cluster_size)`: BFS with 8-connectivity.
   - `frontierGoals(clusters, robot_x, robot_y)`: per cluster, the cell nearest its centroid, sorted by distance to the robot.
   - In `grid.h`, add `findNearestCells(..., n, &out)`. Generalise the ring search without changing what `findNearestCell` returns: make `findNearestCell` a call to `findNearestCells` with n=1 and check that its tests still pass.
3. Rewrite `src/nodes/points_identification.cpp`:
   - subscribe to `/map`, `/safe_map`, `/measure_map`
   - robot pose from `awm::RobotPose`
   - serve `/goal_points`; bad mode → `ROS_WARN` and `return false`; maps missing → empty response with `ROS_WARN_THROTTLE`
   - optional `~dump_maps` writes the map with goals marked `8`
   - delete everything that is dead
4. `test/test_frontiers.cpp`: a known/unknown boundary yields one cluster, a small cluster is filtered out, and goals are sorted by distance.
5. Remove points_identification from the check_format exclusion.

Done when: tests pass. With the sim running (`KEEP_RUNNING=1 scripts/smoke_test.sh`), plus `map_processing` and `points_identification` started with `docker exec -d`, `rosservice call /goal_points "{mode: 1, num: 5}"` returns at most 5 points inside the map, and modes 2 and 3 also answer.
