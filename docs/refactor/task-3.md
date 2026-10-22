# Task 3: map_processing → clean `/safe_map` producer

1. Library `safe_map.h/.cpp`: `makeSafeMap(map, occupied_threshold, caution_threshold, inflate_radius_cells, &safe_map)`. Output values: `-1` unknown, `0` free, `50` caution (uncertain, or within the radius of an obstacle), `100` occupied. Logic from `discretiseMap` + `safeProcessing`, with these bugs fixed:
   - the bounds test is `>= 0` (it was `> 0`)
   - inflation reads the source map but never turns an occupied cell into 50
   Reuse `awm::inflate` where it fits.
2. Library `map_dump.h/.cpp`: `writeMapText(grid, path)` (from `writeMap`; same symbols `* + · space`).
3. Rewrite `src/nodes/map_processing.cpp`:
   - subscribe to `/map` (const ref), publish `/safe_map` latched
   - params `~occupied_threshold` (90), `~caution_threshold` (50), `~inflate_radius_cells` (6), `~dump_maps` (false), `~dump_dir` (`.`)
   - delete files at start-up only when dumps are on
   - no TF and no `/scan_pose`: brain owns that topic
   - no constructor loop, no `wait()`, no print helpers
4. `test/test_safe_map.cpp`: a 10x10 grid with one obstacle checks the band, the borders and unknown cells.
5. Remove map_processing from the check_format exclusion.

Done when: tests pass, and with the sim running (`awm.launch` default args), (`KEEP_RUNNING=1 scripts/smoke_test.sh`), `rosrun awm map_processing` started with `docker exec -d` publishes `/safe_map`, and `rostopic echo -n1 /safe_map/info` matches `/map/info`.
