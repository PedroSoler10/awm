# Task 5: brainT → modular brain that uses the services (both pipelines working separately)

1. Rewrite `src/nodes/brainT.cpp`, using the cleaned brain.cpp as a template:
   - same state machine, statusCallback, RobotPose, `updateMeasureMap`, `writeMeasurementFile`
   - the measure map is built on `/safe_map` (caution cells, value 50, cannot become goals)
   - goal choice: call `/goal_points` with `~goal_mode` (default FUSION), num 1; empty response → `DONE`
   - `ros::service::waitForService` with a timeout and a throttled warning
   - publishes `/measure_map`, `/scan_pose`, goals
   - drop all the listed bugs and dead code (see the table)
   Same node name `brain`, so topics stay identical.
2. New `launch/modular.launch`: starts `map_processing`, `points_identification`, `brainT`. Use it with `roslaunch awm awm.launch brain:=false` in another terminal, or include it behind an arg `modular_test`. Keep `awm.launch` defaults unchanged.
3. README: add a short section "Modular pipeline (experimental)".

Done when: build and tests pass. The standalone smoke test still matches the baseline. The modular run (`TIMEOUT_S=600 EXTRA_LAUNCH=modular.launch scripts/smoke_test.sh brain:=false`) shows ≥3 goal cycles, and, once the known space is measured, at least one frontier goal that grows the map (the `/map` width or height increases).
