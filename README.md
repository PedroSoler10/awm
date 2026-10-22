# awm

ROS Noetic package that makes a TurtleBot3 (Gazebo simulation or real robot) cover a map with measurements: it maps the area with SLAM, drives to the next place that has not been measured yet, stands still to take a measurement, and repeats until nothing is left. The `brain` node runs the state machine and chooses the goals; `move_base` plans the path and drives.

## How it works

```mermaid
flowchart LR
  slam[SLAM<br/>gmapping] -- /map --> mp[map_processing]
  mp -- /safe_map --> brain
  mp -- /safe_map --> pi[points_identification]
  brain -- /measure_map --> pi
  brain -- "/goal_points (service)" --> pi
  brain -- "/move_base (action)" --> mb[move_base]
```

- **`map_processing`** turns the SLAM map into the **safe map**: occupancy ≥ `occupied_threshold` (80) is an obstacle, and free cells closer than `obstacle_margin_m` (0.40 m) to an obstacle are *caution*: no goal is placed there.
- **`brain`** keeps the **measure map** on top of the safe map: every known cell below the threshold is to be measured, *including the caution band* next to walls. A measurement marks every cell within `measure_radius_m` (0.5 m) of the robot.
- **`points_identification`** serves goals on `/goal_points`. **MEASUREMENT** (`2`): the nearest cell still to measure; if it is in the caution band, the goal is the nearest free cell within `measure_radius_m` of it, so measuring there covers it. **EXPLORATION** (`1`): frontiers, one goal per cluster of free cells next to unknown space, minus those already measured. **FUSION** (`3`, default): MEASUREMENT, then EXPLORATION when nothing is left to measure within reach, so the robot also goes to discover unknown space.

`brain` loops: **MEASURING** (stand still `~measure_duration_s`, write a measurement file, update `/measure_map`) → ask `/goal_points` for a goal → **REACHING_GOAL** (`move_base` drives; when it finishes, reached or not, the goal area is marked measured) → MEASURING again → **DONE** when the service has no goal left (`NO MORE GOAL POINTS`). brain then logs how many cells could not be measured (no free cell within `measure_radius_m` of them).

## Quick start

### Native (Ubuntu 20.04 + ROS Noetic)

Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (`ros-noetic-ros-base` is enough), then:

```bash
mkdir -p ~/ros_ws/src && git clone https://github.com/PedroSoler10/awm.git ~/ros_ws/src/awm
```

```bash
~/ros_ws/src/awm/scripts/install_deps.sh
```

```bash
cd ~/ros_ws && source /opt/ros/noetic/setup.bash && catkin_make && source devel/setup.bash
```

```bash
export TURTLEBOT3_MODEL=waffle && roslaunch awm awm.launch
```

`install_deps.sh` installs the apt package of every dependency in `package.xml` (`--print` lists them). It avoids `rosdep`, which cannot resolve Noetic keys on newer rosdistro indexes. The TurtleBot3 launch files need `TURTLEBOT3_MODEL` set in every terminal; add the `source` and `export` lines to `~/.bashrc` to make that permanent.

### Docker

Needs Docker (user in the `docker` group), an X server for Gazebo and RViz, and an Intel or AMD GPU (`/dev/dri`). The workspace stays on the host and is mounted into the container.

```bash
mkdir -p ~/shared_ros_ws/src && git clone https://github.com/PedroSoler10/awm.git ~/shared_ros_ws/src/awm
```

```bash
cd ~/shared_ros_ws/src/awm && docker build -t awm:noetic .
```

```bash
xhost +local:root
```

```bash
docker run -it --rm --name awm --net=host --ipc=host --pid=host --device /dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $HOME/shared_ros_ws:/root/shared_ros_ws awm:noetic
```

Inside the container (ROS, the workspace once built and `TURTLEBOT3_MODEL=waffle` are already loaded):

```bash
cd /root/shared_ros_ws && catkin_make && source devel/setup.bash && roslaunch awm awm.launch
```

- `--net=host --ipc=host --pid=host` let ROS nodes and the GUI work as on the host; `--device /dev/dri`, `DISPLAY` and the X11 socket give the container the screen and the GPU; `--rm` deletes the container on exit (the code lives in the mounted folder).
- A second shell: `docker exec -it awm bash`. Run `xhost -local:root` on the host when you are done.
- Files built in the container belong to root: `sudo chown -R $USER: ~/shared_ros_ws` if needed.
- **Gazebo or RViz crash with `MESA-LOADER` / segfault:** with an NVIDIA GPU replace `--device /dev/dri` by `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all` (needs the NVIDIA Container Toolkit); without a GPU run `export LIBGL_ALWAYS_SOFTWARE=1` before `roslaunch`.

The VS Code dev container, the helper scripts and debugging are described in [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

## Launch

`launch/awm.launch` is the only launch file. Every part can be switched with `name:=value`; `roslaunch awm awm.launch --ros-args` lists them.

```bash
roslaunch awm awm.launch                                  # full simulation
roslaunch awm awm.launch goal_mode:=1                     # frontier goals only
roslaunch awm awm.launch brain:=false gui:=false rviz:=false  # everything but brain, headless
roslaunch awm awm.launch sim:=false                       # real robot (drivers on this machine)
```

| Argument | Default | Description |
|---|---|---|
| `sim` | `true` | Gazebo simulation; `false` starts the real TurtleBot3 drivers (`turtlebot3_bringup`). |
| `model` | `$TURTLEBOT3_MODEL` or `waffle` | `burger`, `waffle`, `waffle_pi`. `TURTLEBOT3_MODEL` must still be set (any value): the included TurtleBot3 launch files require it. |
| `world` | `turtlebot3_world` | World from `turtlebot3_gazebo/worlds`, without `.world`. Simulation only. |
| `gui` | `true` | Show the Gazebo window. Simulation only. |
| `x_pos`, `y_pos`, `z_pos` | `-2.0`, `-0.5`, `0.0` | Spawn position in meters. Simulation only. |
| `slam`, `slam_method` | `true`, `gmapping` | Run SLAM, and which method of `turtlebot3_slam`. |
| `nav` | `true` | Run `move_base`. There is no AMCL: SLAM already localizes the robot. |
| `move_forward_only` | `false` | Navigation never reverses. |
| `goal_xy_tolerance` | `0.10` | Distance (m) at which `move_base` counts a goal as reached (TurtleBot3 default `0.05`). Measuring covers 0.5 m, so 10 cm is harmless. |
| `goal_yaw_tolerance` | `3.15` | Heading error (rad) accepted at the goal. `3.15` ignores the heading, which measuring does not need; tight values (TurtleBot3 default `0.17`) made the robot back up to re-approach close goals. |
| `rviz`, `rviz_config` | `true`, `rviz/awm.rviz` | Open RViz with this configuration. `awm.rviz` is the TurtleBot3 navigation view plus an **AWM** group with `/safe_map` (costmap colours, caution band visible) and `/measure_map` (white measured, black blocked, grey still to measure), both half transparent over `/map`; untick them in the Displays panel. |
| `brain` | `true` | Run `brain`. With `false` the other awm nodes still run, so brain can be started by hand (`rosrun awm brain`) or under gdb. |
| `goal_mode` | `3` | `/goal_points` mode brain asks for: 1 EXPLORATION, 2 MEASUREMENT, 3 FUSION. |
| `occupied_threshold` | `80` | SLAM occupancy from this value up is an obstacle; every known cell below it is measured. |
| `obstacle_margin_m` | `0.40` | No goal closer than this to an obstacle (cells exactly this far are allowed). |
| `measure_radius_m` | `0.5` | Cells this close to a measurement count as measured (brain and points_identification). |

## Nodes, topics and parameters

| Name | Type | From | To |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | SLAM | `map_processing` |
| `/safe_map` | `nav_msgs/OccupancyGrid`, latched | `map_processing` | `brain`, `points_identification` |
| `/measure_map` | `nav_msgs/OccupancyGrid`, latched | `brain` | `points_identification` |
| `/scan_pose` | `geometry_msgs/PoseStamped` | `brain` | RViz, tools |
| `/move_base` | `move_base_msgs/MoveBase` action | `brain` (client) | `move_base` (server) |
| `/goal_points` | `awm/GoalPoints` service | `points_identification` (server) | `brain` |

Map values: SLAM `-1` unknown, `0..100` occupancy. Safe map: `-1` unknown, `0` free, `50` caution (too close to an obstacle for a goal), `100` occupied. Measure map: `100` unknown or occupied (never measured), `0` measured, `-1` still to measure.

Private parameters (set with `rosrun awm <node> _name:=value`; `awm.launch` sets the ones that have a launch argument):

| Node | Parameter | Default | Meaning |
|---|---|---|---|
| `map_processing` | `~occupied_threshold` | `80` | From this occupancy up: obstacle. |
| | `~obstacle_margin_m` | `0.40` | Free cells closer than this to an obstacle become caution. |
| | `~dump_maps`, `~dump_dir` | `false`, `.` | Write `map.txt` and `safe_map.txt` on every map. |
| `brain` | `~goal_mode` | `3` | Mode sent to `/goal_points`. Other values stop the node. |
| | `~measure_duration_s` | `1.0` | Time standing still per measurement. |
| | `~measure_radius_m` | `0.5` | Cells this close to the robot count as measured. |
| | `~output_dir` | `.` | Folder for the measurement files (`~/.ros` under roslaunch). |
| `points_identification` | `~measure_radius_m` | `0.5` | Must match brain's: how far from a caution cell its goal may be. |
| | `~search_limit_cells` | `200` | Farthest cell still to measure searched from the robot. |
| | `~min_cluster_size` | `5` | Smaller frontier clusters are ignored. |
| | `~dump_maps`, `~dump_dir` | `false`, `.` | Write `safe_map_with_goals.txt` (goals drawn as `8`) on every request. |

Try the service by hand: `rosservice call /goal_points "{mode: 2, num: 5}"` (`num: 0` returns all). An unknown mode makes the call fail; missing maps or pose give an empty list.

## Code layout

```
include/awm/, src/lib/   awm_core library (namespace awm): grid helpers, measure map, safe map,
                         frontiers, map text dump, robot pose from tf, /goal_points client
src/nodes/               brain, map_processing, points_identification (thin: params, topics, calls)
test/                    gtest unit tests of awm_core, no roscore needed
srv/GoalPoints.srv       the /goal_points service
launch/awm.launch        the only launch file
rviz/awm.rviz            RViz view with /map, /safe_map and /measure_map
scripts/                 install_deps, check_format, container_build, smoke_test, debug_brain
docs/DEVELOPMENT.md      style, tools, dev container, debugging
```

## Tests

Unit tests (from the workspace, natively or in the container):

```bash
catkin_make run_tests_awm && catkin_test_results build/test_results/awm
```

They cover the grid helpers, safe map, measure map and measurement goals, and frontiers. The `/goal_points` client and the nodes need ROS running and are covered by the smoke test, which runs a headless simulation in the dev container until `brain` has made 3 measurements and prints `PASS` or `FAIL` (see [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md#scripts)):

```bash
scripts/smoke_test.sh goal_mode:=2
```

## Known limitations

- **The measurement is a placeholder:** an empty file named after the position and time (`<x>_<y>_<time>.txt`).
- **gmapping publishes only `-1`, `0` and `100`**, so `occupied_threshold` makes no difference with it. The nodes handle `0..100` probabilities, but Cartographer, the probabilistic SLAM TurtleBot3 supports, has no Noetic binary package and would have to be built from source.
- **A measurement ignores walls:** it marks every cell within `measure_radius_m`. In a corridor too narrow for a free cell on the near side, a cell next to a thin wall can be measured from the other side of it.
- **Ring search sampling:** the nearest-cell search samples 360 angles per circle, so beyond about 57 cells (≈2.9 m) it can skip single cells between two samples. They are found once the robot gets closer.
- **The service call blocks brain's loop** until `/goal_points` answers (it waits for the service if `points_identification` is down), so `/scan_pose` pauses meanwhile.
- **No goal timeout:** `brain` relies on `move_base` aborting a goal it cannot reach.
