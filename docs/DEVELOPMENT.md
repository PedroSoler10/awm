# Development

How the code is organised and styled, the tools that check it, the dev container, the helper scripts and debugging. Installing and running the package is in the [README](../README.md).

## Target platform

**ROS Noetic on Ubuntu 20.04** (ROS 1: `ros/ros.h`, `actionlib`, `catkin`), **C++14**. ROS 2 distributions do not provide these headers, so editor tools must use the Noetic environment; the dev container below provides it.

## Code layout

- **Library `awm_core`** (`include/awm/*.h`, `src/lib/*.cpp`, namespace `awm`): all the logic on grids and poses. Functions are pure where possible: they take `nav_msgs::OccupancyGrid`s and numbers, do not publish and do not read ROS time, so they are unit tested without a ROS master. The exceptions are `RobotPose` (tf) and `requestGoal` (service call).
- **Nodes** (`src/nodes/<name>.cpp`, listed in `AWM_EXECUTABLES` in `CMakeLists.txt`): one class per node. The constructor reads parameters and sets up publishers, subscribers and services; `main` calls `ros::spin()`, or `run()` for `brain`'s 10 Hz control loop. Map publishers are latched with queue 1, so a node that starts later still gets the last map.
- **Who decides what:** the safe map says where the robot may go (goals only on free cells), the measure map says what is measured; `points_identification` combines them into goals and `brain` only runs the state machine.
- **Parameters:** private, read with `pnh.param(name, member, default)`, so the defaults live in the code. An invalid value logs `ROS_FATAL` and stops the node at start-up. The README lists them all.
- **Tests** (`test/test_<name>.cpp`): gtest on small hand-made grids with exact cell values, registered in section 6 of `CMakeLists.txt`.
- **Interfaces:** `srv/GoalPoints.srv` is the only one. A library file that includes a generated header needs the `add_dependencies(awm_core ...)` that `CMakeLists.txt` already has.
- **Dependencies:** every ROS package whose header is included goes both in `find_package(catkin ...)` and as a `<depend>` in `package.xml`; `scripts/install_deps.sh` and the `Dockerfile` install from `package.xml`.

## Coding style

The [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide) (based on Google's), enforced by `.clang-format` and `CPPLINT.cfg`:

- 2-space indent, attached braces, 100 columns.
- Includes in three blocks: `.h` headers (ROS, C), then C++ standard headers, then `"awm/..."`; a library `.cpp` includes its own header first.
- Header guards `AWM_<NAME>_H_` (`CPPLINT.cfg` sets `root=include`).
- Names: `UpperCamelCase` types, `lowerCamelCase` functions, `snake_case` variables, members with a trailing `_`, constants `kName`, library code in namespace `awm`.
- A function that can fail returns `bool` and writes its result through an out-pointer, leaving it unchanged on failure (no `std::optional` in C++14). Large messages go by `const&`; out-parameters are pointers, so the call site shows what is written.
- No `using namespace std`, no `#define` constants, no blocking console input. Comments explain *why*; no commented-out code.
- The build uses `-Wall -Wextra`; keep it warning-free.

Check, or fix, from the package folder (needs `sudo apt install clang-format cpplint` on the host; they are not in the image):

```bash
scripts/check_format.sh
```

`scripts/check_format.sh --fix` applies the formatter; review with `git diff`. If cpplint and clang-format disagree, fix the config files, not the code.

## VS Code and the dev container

`.devcontainer/devcontainer.json` runs the editor inside the image built from the `Dockerfile`, so IntelliSense resolves the Noetic headers. Install the **Dev Containers** extension, open the `awm` folder and run **Dev Containers: Reopen in Container**. The container mounts the whole `shared_ros_ws` at `/root/shared_ros_ws`, opens the package at `/root/shared_ros_ws/src/awm`, and has gdb, gtest and headless Gazebo.

In the image, `/root/ros_env.sh` sources ROS and the workspace (once built); `/root/.bashrc` sources it and `BASH_ENV` points to it, so interactive shells, `docker exec` and `bash -c` all have ROS loaded. `TURTLEBOT3_MODEL=waffle` is set with `ENV`.

For format on save, install `xaver.clang-format` and add to the user settings:

```json
"[cpp]": { "editor.defaultFormatter": "xaver.clang-format", "editor.formatOnSave": true }
```

## Scripts

| Script | Runs on | Does |
|---|---|---|
| `scripts/install_deps.sh [--print]` | host or image | Installs the apt packages of `package.xml` |
| `scripts/check_format.sh [--fix]` | host | clang-format and cpplint on every tracked C++ file |
| `scripts/container_build.sh [--test]` | host | `catkin_make` (and the unit tests) inside the existing dev container |
| `scripts/smoke_test.sh [launch args]` | host | Headless simulation in the dev container until `brain` makes 3 measurements; prints nodes, brain events, errors and `PASS`/`FAIL` |
| `scripts/debug_brain.sh [functions]` | container | `brain` under gdb with breakpoints |

The container scripts find the dev container by its VS Code label (its name is random), start it if it is stopped, and never rebuild the image. `smoke_test.sh` options: `MIN_CYCLES=5`, `TIMEOUT_S=600`, `KEEP_RUNNING=1` (stop later with `scripts/smoke_test.sh --stop`); the log stays in the container at `/tmp/awm_sim.log`.

## Debugging brain with gdb

Build with debug symbols (otherwise variables show `<optimized out>`) and start everything except `brain`; pausing Gazebo stops `/clock`, so the whole system waits while you look:

```bash
cd ~/shared_ros_ws && catkin_make -DCMAKE_BUILD_TYPE=Debug
```

```bash
roslaunch awm awm.launch brain:=false gui:=false rviz:=false
```

```bash
rosservice call /gazebo/pause_physics
```

**Terminal:** `scripts/debug_brain.sh` stops at `Brain::control`; pass other functions to stop there instead (`scripts/debug_brain.sh Brain::sendGoal awm::findNearestCell`). If gdb stops on a function's opening line, `next` once before `print`ing members. Useful commands: `next`/`step`, `finish`, `continue`, `print x`/`display x`, `bt`, `break f if cond`, `watch x`.

**VS Code:** in the dev container run the task **sim without brain** (optionally **pause sim**), set a breakpoint, and press F5 with **Debug brain (launch)** (builds in Debug and starts `brain`) or **Debug brain (attach)** (attaches to a running `brain`). If the container predates gdb, run **Dev Containers: Rebuild Container** once.

## Docker housekeeping

`docker ps -a` and `docker images` list containers and images; remove them with `docker rm awm` and `docker rmi awm:noetic`, and dangling layers with `docker image prune`.
