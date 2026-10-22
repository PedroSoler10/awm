# Task 1: Remove files with no value

1. `git rm` these files: `src/mapperT.cpp`, `src/mapper.cpp`, `src/myTest.cpp`, `src/measure_server.cpp`, `src/service_core.cpp`, `msg/Num.msg`.
2. `CMakeLists.txt`: remove `mapper` and `mapperT` from `AWM_EXECUTABLES`, remove `add_message_files` (keep `GoalPoints.srv`), and remove the "Not built on purpose" comment block.
3. In the README, remove or fix any mention of the deleted files.
4. Build. Commit: "Remove duplicate, broken and placeholder sources". In the commit body, give the one-line reason for each file, taken from the table in PLAN.md.

Done when: exactly 4 executables build (brain, brainT, map_processing, points_identification), and the sim smoke test matches the baseline.
