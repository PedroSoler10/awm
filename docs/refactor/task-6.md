# Task 6: One brain, mode chosen by a parameter (final structure)

1. In `src/nodes/brain.cpp`, add the interface `class GoalSource { public: virtual ~GoalSource() = default; virtual bool next(const Context& ctx, geometry_msgs::PoseStamped* goal) = 0; };`, where `Context` has the robot pose and the measure map. Two implementations, each in its own header/source under `include/awm/` and `src/lib/`:
   - `NearestUnmeasuredGoalSource`: today's `pubGoal` search.
   - `ServiceGoalSource`: brainT's service call, with `~goal_mode`.
   brain owns a `std::unique_ptr<GoalSource>` created from `~goal_source` (`internal` | `service`, default `internal`; an unknown value → `ROS_FATAL` and exit). In service mode, brain subscribes to `/safe_map` and builds the measure map on it. In internal mode it uses `/map` as today.
2. Delete `src/nodes/brainT.cpp` and `launch/modular.launch`, and remove `brainT` from CMake.
3. `awm.launch`: add the arg `mode` (default `standalone`, or `modular`). The brain node gets `<param name="goal_source" value="internal|service">`, and `map_processing` + `points_identification` start inside `<group if="$(eval mode == 'modular')">`. Keep `brain:=false` working.
4. Unit test the internal GoalSource on a hand-built measure map. The service source is checked in the sim.
5. README: "Modes" section with a table (mode → nodes started → goal policy), and update the topic list.

Done when: both `scripts/smoke_test.sh` and `TIMEOUT_S=600 scripts/smoke_test.sh mode:=modular` pass, and standalone still matches the baseline.
