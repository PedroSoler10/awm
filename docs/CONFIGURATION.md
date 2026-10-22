# Project configuration

This page explains how the project is configured: the coding style, the tools that enforce it, the editor setup, and the development container. For installing and running the package, see the [README](../README.md).

## Target platform

The package targets **ROS Noetic on Ubuntu 20.04** (ROS 1). Its code uses the ROS 1 API (`ros/ros.h`, `actionlib`, `catkin`). ROS 2 distributions such as Jazzy do not provide these headers, so editor tools must use the Noetic environment. The development container described below provides it.

## Coding style

The style follows the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide), which is based on the Google C++ Style Guide. Three configuration files set the rules:

| File | Used by | Sets |
|---|---|---|
| `.clang-format` | clang-format | Indentation, braces, line length, include sorting |
| `CPPLINT.cfg` | cpplint | Maximum line length |
| `scripts/check_format.sh` | Command line | Runs both tools over every tracked C++ file in `src/`, `include/` and `test/` |

The key rules:

- **Indentation:** 2 spaces. No tabs.
- **Braces:** the opening `{` stays at the end of the line, as in `class Brain {`.
- **Line length:** at most 100 characters. Both tools use the same limit.
- **Include order:** `.h` headers (ROS, TF2, and other C headers) come first, sorted alphabetically. C++ standard library headers without `.h` (`<string>`, `<cmath>`) come after, in their own block. Headers of this package (`"awm/grid.h"`) come last, in their own block; a library `.cpp` includes its own header first.
- **Whitespace:** no trailing spaces, and one blank line between logical blocks.
- **Comments:** explain *why* a decision was made, not what the code does. No commented-out code.
- **Header guards:** `AWM_<NAME>_H_` (`CPPLINT.cfg` sets `root=include`, so cpplint expects this form).

Naming and C++ rules (the ROS style guide, as applied in this package):

| What | Rule | Example |
|---|---|---|
| Classes, structs | `UpperCamelCase` | `RobotPose`, `GridCell` |
| Functions and methods | `lowerCamelCase`, no `fn` prefix | `updateMeasureMap`, `pubGoal` |
| Variables | `snake_case` | `goal_status`, `robot_x` |
| Class members | `snake_case` with a trailing `_` | `measure_map_`, `pub_goal_` |
| Constants | `kName` | `kMeasured`, `kSafeCaution` |
| Namespace | `awm` for all library code | `awm::findNearestCell` |

- **C++14** (the Noetic toolchain), so no `std::optional`: a function that can fail returns `bool` and writes its result through an out-pointer, leaving it unchanged on failure (`bool lookup(geometry_msgs::PoseStamped* out)`).
- Maps and other large messages are passed by `const&`; out-parameters are pointers, so the call site shows what is written (`&measure_map`).
- No `using namespace std`, no `#define` constants (`M_PI`, `constexpr`), no `scanf` or other blocking console input.

## Code layout

- **Library `awm_core`** (`include/awm/*.h`, `src/lib/*.cpp`, namespace `awm`): all the logic that works on grids and poses. Functions are pure where possible: they take `nav_msgs::OccupancyGrid`s and plain numbers, do not publish, and do not read ROS time, so they can be unit tested without a ROS master. The exceptions are `RobotPose` (tf) and `ServiceGoalSource` (service call), which need a running node.
- **Nodes** (`src/nodes/<name>.cpp`, one executable each, listed in `AWM_EXECUTABLES` in `CMakeLists.txt`): one class per node. The constructor only reads parameters and sets up publishers, subscribers and services. `main` calls `ros::spin()`, or the class's `run()` when there is a control loop (`brain`, 10 Hz). Map publishers are latched, so a node that starts later still gets the last map.
- **Parameters:** every tunable value is a private parameter read with `ros::NodeHandle pnh("~")` and `pnh.param(name, member, default)`, so the defaults live in the code and `rosrun awm <node> _name:=value` overrides them. An invalid value logs `ROS_FATAL` and shuts the node down at start-up instead of running with a wrong setting. The README lists every parameter.
- **Tests** (`test/test_<name>.cpp`): gtest, one file per library header group, registered with `catkin_add_gtest` in section 6 of `CMakeLists.txt`. They build small grids by hand and check exact cell values. See the README, "Unit tests", for how to run them.
- **Interfaces:** `srv/GoalPoints.srv` is the only custom interface. A library source that includes a generated header (`awm/GoalPoints.h`) needs `add_dependencies(awm_core ${${PROJECT_NAME}_EXPORTED_TARGETS})`, which `CMakeLists.txt` already has.
- **Dependencies:** every ROS package whose header is included must be listed both in `find_package(catkin ...)` in `CMakeLists.txt` and as a `<depend>` in `package.xml`; `scripts/install_deps.sh` installs from `package.xml`.

## Checking the style

From the package folder, run:

```bash
scripts/check_format.sh
```

This prints the formatter and lint errors without changing any file. To apply the formatter's fixes automatically, run:

```bash
scripts/check_format.sh --fix
```

Review the changes with `git diff` afterwards. The script needs `clang-format` and `cpplint` on the `PATH`. On Ubuntu:

```bash
sudo apt install clang-format cpplint
```

## VS Code setup

Install these extensions:

- **clang-format** (`xaver.clang-format`): formats the file on save, using `.clang-format`.
- **C/C++** (`ms-vscode.cpptools`): IntelliSense and inline error squiggles.
- **cpplint**: shows lint messages in the Problems panel (Ctrl+Shift+M).

Add this to your user settings to format C++ files on save:

```json
"[cpp]": {
  "editor.defaultFormatter": "xaver.clang-format",
  "editor.formatOnSave": true
}
```

If a lint message says the same thing the formatter keeps changing, the two configuration files disagree. Fix the config files rather than the code.

## Development container

The `.devcontainer/devcontainer.json` file runs the editor inside the same Docker image as the Docker option of the README. This gives IntelliSense the Noetic headers, so `ros/ros.h` and the other ROS types resolve.

1. Install the **Dev Containers** extension (`ms-vscode-remote.remote-containers`).
2. Open the `awm` folder in VS Code.
3. Run **Dev Containers: Reopen in Container** from the command palette (Ctrl+Shift+P).

The container:

- builds from the [`Dockerfile`](../Dockerfile), with the package folder as the build context;
- mounts the whole `shared_ros_ws` folder at `/root/shared_ros_ws`, which the Dockerfile expects;
- opens the package at `/root/shared_ros_ws/src/awm`.

To follow the build, open the **Output** panel and select **Dev Containers**.

The image has gdb, googletest and rosunit/rostest, and Gazebo can run headless in it, so the build, the unit tests and the smoke test all run there. `scripts/container_build.sh` and `scripts/smoke_test.sh` find the container by its VS Code label (its name is random) and reuse it; they never rebuild the image. clang-format and cpplint are not in the image: run `scripts/check_format.sh` on the host.

## Files in this package

| Path | Purpose |
|---|---|
| `include/awm/`, `src/lib/` | Library `awm_core` |
| `src/nodes/` | The nodes, one executable per file |
| `test/` | Unit tests |
| `srv/GoalPoints.srv` | The `/goal_points` service |
| `launch/awm.launch` | The only launch file |
| `.clang-format` | Formatter rules |
| `CPPLINT.cfg` | Lint settings |
| `scripts/check_format.sh` | Format and lint check |
| `.devcontainer/devcontainer.json` | Development container |
| `Dockerfile` | Base image with all dependencies |
| `scripts/install_deps.sh` | Installs the dependencies in `package.xml` |
| `scripts/container_build.sh` | Builds (and with `--test`, tests) inside the existing dev container, from the host |
| `scripts/smoke_test.sh` | Headless simulation test inside the dev container, prints `PASS` or `FAIL` |
| `scripts/debug_brain.sh`, `.vscode/` | Debugging `brain` with gdb |
