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
| `scripts/check_format.sh` | Command line | Runs both tools over the tracked C++ files |

The key rules:

- **Indentation:** 2 spaces. No tabs.
- **Braces:** the opening `{` stays at the end of the line, as in `class Brain {`.
- **Line length:** at most 100 characters. Both tools use the same limit.
- **Include order:** `.h` headers (ROS, TF2, and other C headers) come first, sorted alphabetically. C++ standard library headers without `.h` (`<string>`, `<cmath>`) come after, in their own block.
- **Whitespace:** no trailing spaces, and one blank line between logical blocks.
- **Comments:** explain *why* a decision was made, not what the code does.

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

## Files in this package

| Path | Purpose |
|---|---|
| `.clang-format` | Formatter rules |
| `CPPLINT.cfg` | Lint settings |
| `scripts/check_format.sh` | Format and lint check |
| `.devcontainer/devcontainer.json` | Development container |
| `Dockerfile` | Base image with all dependencies |
| `scripts/install_deps.sh` | Installs the dependencies in `package.xml` |
