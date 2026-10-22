# awm

ROS Noetic nodes, interfaces and launch files for the AWM project, running a TurtleBot3 in simulation or on the real robot.

## Contents

- `src/nodes/`: C++ nodes, one executable per file (`brain`, `brainT`, `map_processing`, `points_identification`).
- `include/awm/`, `src/lib/`: the `awm_core` library shared by the nodes (grid helpers, robot pose from tf, measure map).
- `test/`: unit tests of `awm_core` (gtest, no roscore needed; run with `scripts/container_build.sh --test`).
- `srv/`: custom interface (`GoalPoints.srv`).
- `launch/awm.launch`: single launch file for the simulation or the real robot, with a switch for each module (see [Launch options](#launch-options)).
- `Dockerfile`: builds an image with every dependency already installed.
- `scripts/debug_brain.sh`, `.vscode/`: debug `brain` with gdb (see [Debugging with gdb](#debugging-with-gdb)).
- `scripts/container_build.sh`, `scripts/smoke_test.sh`: build and run a headless simulation test from the host, inside the dev container (see [Build and smoke test from the host](#build-and-smoke-test-from-the-host)).
- `docs/CONFIGURATION.md`: coding style, formatting and lint tools, editor setup, and the development container.

## Option A: Native install (Ubuntu 20.04 + ROS Noetic)

### 1. Install ROS Noetic

Follow the official instructions at http://wiki.ros.org/noetic/Installation/Ubuntu (`ros-noetic-ros-base` is enough).

### 2. Create a workspace and link the package

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/PedroSoler10/awm.git awm
```

### 3. Install the dependencies

Install the packages listed in `package.xml`. The script reads that file and installs the matching apt packages, so the list is not repeated here. It uses apt directly, which avoids `rosdep` because `rosdep` can't resolve ROS Noetic keys on newer rosdistro indexes.

```bash
cd ~/ros_ws/src/awm && ./scripts/install_deps.sh
```

To see the packages without installing them, run `./scripts/install_deps.sh --print`.

### 4. Build

```bash
cd ~/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 5. Launch

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch awm awm.launch
```

This starts the whole simulation. Other setups are described in [Launch options](#launch-options).

`TURTLEBOT3_MODEL` selects the robot model spawned in Gazebo. Set it to `waffle` in every terminal that runs the simulation or the teleop. The launch files in this package default to `waffle`, but the TurtleBot3 launch files they include require the variable, so the simulation fails to start if it is unset.

To have this set up in every new terminal, add the lines to your `~/.bashrc` once:

```bash
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc && echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
```

### Updating

Update the code with git and rebuild the workspace:

```bash
cd ~/ros_ws/src/awm && git pull
cd ~/ros_ws && catkin_make
```

## Option B: Docker

Edit the code on your machine and build it inside a container. Dependencies live in the image, so nothing needs to be installed on your machine except Docker.

### Requirements

- **Docker** installed, and your user in the `docker` group so you can run it without `sudo` (`sudo usermod -aG docker $USER`, then log out and back in).
- **Internet access** the first time you build: the base image `ros:noetic-ros-base` and the apt packages are downloaded then.
- **An X server** on the host for the GUI (Gazebo and RViz). On Ubuntu it is usually already running; `xhost` comes from the `x11-xserver-utils` package.
- **An Intel or AMD GPU** (the `/dev/dri` device) for the GUI. With an NVIDIA GPU, or if `/dev/dri` does not exist, see [Troubleshooting](#troubleshooting-gazebo-or-rviz-crash-on-start).

### 1. Get the code

```bash
mkdir -p ~/shared_ros_ws/src && git clone https://github.com/PedroSoler10/awm.git ~/shared_ros_ws/src/awm
```

### 2. Build the image

The image only needs rebuilding when the dependencies in the `Dockerfile` change:

```bash
cd ~/shared_ros_ws/src/awm && docker build -t awm:noetic .
```

- `cd ~/shared_ros_ws/src/awm` goes into the cloned package, which contains the `Dockerfile`.
- `docker build` runs the `Dockerfile` and creates the image. `-t awm:noetic` gives it the name `awm:noetic`. The `.` is the build context, the folder Docker copies from.

### 3. Start the container

The folder `~/shared_ros_ws` on your machine is mounted at `/root/shared_ros_ws` in the container. The container is removed when you exit it, so everything you keep lives on your machine.

Allow the container to open windows on your display (Gazebo and RViz need this), then start it:

```bash
xhost +local:root
```

```bash
docker run -it --rm --name awm --net=host --ipc=host --pid=host --device /dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $HOME/shared_ros_ws:/root/shared_ros_ws awm:noetic
```

- `docker run awm:noetic` starts a container from the image.
- `-it` keeps an interactive terminal open, so you get a shell inside the container.
- `--rm` deletes the container when you exit. Your code is safe because it lives in the mounted folder.
- `--name awm` names the container, so other commands can refer to it (for example `docker exec -it awm bash`).
- `--net=host`, `--ipc=host` and `--pid=host` share the host's network, shared memory and processes with the container. ROS nodes find each other and Gazebo and RViz run without extra setup.
- `--device /dev/dri` gives the container access to the graphics card, which Gazebo and RViz need to draw their windows.
- `-e DISPLAY=$DISPLAY` passes your screen's address into the container.
- `-v /tmp/.X11-unix:/tmp/.X11-unix:rw` shares the X11 socket, which is how the container's windows appear on your screen.
- `-v $HOME/shared_ros_ws:/root/shared_ros_ws` mounts your workspace into the container. Editing on your machine and building in the container use the same files.

After you exit the container, close the display access:

```bash
xhost -local:root
```

### 4. Build the workspace

Inside the container, run this the first time and again after every code change:

```bash
cd /root/shared_ros_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

- `cd /root/shared_ros_ws` moves to the workspace, the folder that contains `src/awm`.
- `source /opt/ros/noetic/setup.bash` loads ROS, which `catkin_make` needs.
- `catkin_make` builds the package and creates `build/` and `devel/` in the workspace. These folders appear in `~/shared_ros_ws` on your machine.
- `source devel/setup.bash` makes the new `awm` package visible to `roslaunch` in this shell. Shells opened later load it automatically (see [Setting up .bashrc](#setting-up-bashrc)).

The workspace is only loaded in new shells once it has been built, so shells opened before the first `catkin_make` need the `source devel/setup.bash` above.

### 5. Launch the simulation

Inside the container:

```bash
roslaunch awm awm.launch
```

- `roslaunch awm awm.launch` starts the whole simulation: the Gazebo world, the TurtleBot3, SLAM, navigation, RViz and the `brain` node.
- `TURTLEBOT3_MODEL` selects the robot model, which is `waffle`. You don't need to export it: the `Dockerfile` sets it with `ENV`, so every terminal in the container already has it.

Gazebo and RViz should open. To drive the robot, open a second terminal in the container:

```bash
docker exec -it awm bash
```

- `docker exec -it awm bash` opens a new shell in the running container `awm`. The first terminal is busy with the simulation, so you need a second one.

Then run the teleop:

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

- `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` starts keyboard control of the robot. The new shell already has ROS, the workspace and `TURTLEBOT3_MODEL` loaded, because `/root/.bashrc` sets them up (see [Setting up .bashrc](#setting-up-bashrc)).

### Troubleshooting: Gazebo or RViz crash on start

If Gazebo or RViz crash right after starting, with `libGL error: MESA-LOADER: failed to retrieve device information` followed by a segmentation fault, the container cannot use your graphics card.

- **Intel or AMD GPU:** check that the `docker run` command from [step 3](#3-start-the-container) includes `--device /dev/dri`.
- **NVIDIA GPU:** replace `--device /dev/dri` with `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all`. This needs the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed on your machine.
- **No GPU device (`/dev/dri` does not exist), or the options above do not work:** render in software, which is slower. Run this in the container terminal before `roslaunch`:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Editing and rebuilding

1. Edit the files in `~/shared_ros_ws/src/awm` with your usual editor on your machine.
2. Run `catkin_make` inside the container (step 4).
3. Relaunch with `roslaunch` (step 5).

Files created by `catkin_make` inside the container are owned by root, so you may need `sudo` to change or delete `build/` and `devel/` on your machine. You can fix the ownership with `sudo chown -R $USER: ~/shared_ros_ws`.

To update the code, run `git pull` in `~/shared_ros_ws/src/awm` on your machine and rebuild inside the container.

### Removing Docker images and containers

When you no longer need the image or a container, list them first:

```bash
docker ps -a
docker images
```

Remove a stopped container:

```bash
docker rm awm
```

Remove the image:

```bash
docker rmi awm:noetic
```

To delete unused leftovers (dangling layers from old builds) and free disk space:

```bash
docker image prune
```

## Launch options

`launch/awm.launch` is the only launch file. It starts the robot and every module of the project, and each part can be switched on or off with a `name:=value` argument added to the command. The commands work the same natively and inside the container.

### Examples

```bash
# Full simulation: Gazebo, SLAM, navigation, RViz and the brain node
roslaunch awm awm.launch

# Simulation without RViz and without the brain node
roslaunch awm awm.launch rviz:=false brain:=false

# Simulation without the Gazebo window (the Gazebo server still runs)
roslaunch awm awm.launch gui:=false

# Different world and robot start position
roslaunch awm awm.launch world:=turtlebot3_house x_pos:=-3.0 y_pos:=1.0

# SLAM only: no navigation and no brain node
roslaunch awm awm.launch nav:=false brain:=false

# Real robot: starts the robot drivers on this machine instead of Gazebo
roslaunch awm awm.launch sim:=false
```

To see every argument with its default and description, run:

```bash
roslaunch awm awm.launch --ros-args
```

### Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `sim` | `true` | `true` starts Gazebo, `false` starts the real TurtleBot3 drivers (`turtlebot3_bringup`) on this machine. |
| `model` | `$TURTLEBOT3_MODEL`, or `waffle` | `burger`, `waffle` or `waffle_pi`. See the note below. |
| `world` | `turtlebot3_world` | Gazebo world from `turtlebot3_gazebo/worlds`, without `.world` (for example `turtlebot3_house`). Simulation only. |
| `gui` | `true` | Show the Gazebo window. Simulation only. |
| `x_pos`, `y_pos`, `z_pos` | `-2.0`, `-0.5`, `0.0` | Where the robot appears in the world, in meters. Simulation only. |
| `slam` | `true` | Run SLAM. |
| `slam_method` | `gmapping` | SLAM method used when `slam:=true`. |
| `nav` | `true` | Run AMCL and `move_base`. |
| `move_forward_only` | `false` | Navigation never reverses the robot. |
| `rviz` | `true` | Open RViz. |
| `rviz_config` | `turtlebot3_navigation.rviz` | RViz configuration file, as a full path. |
| `brain` | `true` | Run the `brain` node. |

The arguments are the only configuration: the nodes in this package do not read a parameter file.

**`TURTLEBOT3_MODEL` must be set, but `model` decides the robot.** The TurtleBot3 launch files that `awm.launch` includes fail to start if the `TURTLEBOT3_MODEL` environment variable is not set, even though `awm.launch` passes them the model. Any valid value is enough, and the `model` argument decides which robot is used:

```bash
roslaunch awm awm.launch model:=burger
```

Inside the container the variable is already set to `waffle` (in the `Dockerfile`), so this command works as it is. Natively, run `export TURTLEBOT3_MODEL=waffle` first (step 5 of the install).

### Running nodes separately

The `brain` node does not need to start with the rest. To run it in its own terminal, for example to restart it without restarting Gazebo, start the rest with `brain:=false` and run the node by itself:

```bash
roslaunch awm awm.launch brain:=false
```

```bash
rosrun awm brain
```

`brain` reads these private parameters (set them with `_name:=value` after `rosrun awm brain`):

| Parameter | Default | Meaning |
|---|---|---|
| `~measure_duration_s` | `3.0` | Time standing still per measurement |
| `~measure_radius_m` | `0.5` | Cells this close to the robot count as measured |
| `~inflate_radius_cells` | `6` | Safety margin around obstacles, in cells |
| `~search_limit_cells` | `200` | Farthest goal searched from the robot, in cells |
| `~occupied_threshold` | `90` | Occupancy above this is an obstacle |
| `~output_dir` | `.` | Folder for the measurement files (`.` is the working directory, `~/.ros` under roslaunch) |

### Build and smoke test from the host

When the VS Code dev container exists (see [docs/CONFIGURATION.md](docs/CONFIGURATION.md)), two scripts use it from a host terminal. They find the container by its VS Code label and start it if it is stopped. They never rebuild the image.

```bash
scripts/container_build.sh
```

- Runs `catkin_make` inside the container. Add `--test` to also run the unit tests.

```bash
scripts/smoke_test.sh
```

- Starts `awm.launch` without the Gazebo window and RViz, waits until `brain` has made 3 measurements, prints the nodes, the brain events and any errors, then stops the simulation and prints `PASS` or `FAIL`.
- Arguments go to `awm.launch` (for example `scripts/smoke_test.sh world:=turtlebot3_house`).
- `MIN_CYCLES=5` asks for more measurements, `TIMEOUT_S=600` waits longer, and `KEEP_RUNNING=1` leaves the simulation running for manual checks; stop it with `scripts/smoke_test.sh --stop`.

## Debugging with gdb

gdb stops the `brain` node at a line you choose, so you can read variables and step through the code. It is installed in the Docker image (natively: `sudo apt install gdb`). Both ways below need the same preparation.

**Preparation (once per build).** Build with debug symbols, otherwise variables show `<optimized out>`:

```bash
cd ~/shared_ros_ws && catkin_make -DCMAKE_BUILD_TYPE=Debug
```

Start everything except `brain` and, if you want the world to wait while you look, pause it:

```bash
roslaunch awm awm.launch brain:=false gui:=false rviz:=false
```

```bash
rosservice call /gazebo/pause_physics
```

Unpause with `rosservice call /gazebo/unpause_physics`. Pausing stops `/clock`, so the robot, SLAM and the node's timers all wait.

### In the terminal

```bash
scripts/debug_brain.sh
```

It starts `brain` under gdb and stops at `Brain::control`. Pass other functions to stop at them: `scripts/debug_brain.sh Brain::pubGoal Brain::measure`, or library functions such as `awm::findNearestCell`.

gdb may stop on the function's opening line, before `this` is set, so `print state_` fails with "Cannot access memory". Type `next` once and it works.

| Command | What it does |
|---|---|
| `next` / `step` | Run the next line / enter the function called on it |
| `finish` | Run until the current function returns |
| `continue` | Resume until the next breakpoint |
| `print x`, `display x` | Show a variable once / every time gdb stops |
| `bt`, `up`, `down` | Call stack and moving between its frames |
| `break f if cond` | Stop in `f` only when `cond` is true |
| `watch x` | Stop whenever `x` changes |
| `Ctrl+C` | Interrupt the running program |
| `quit` | Leave gdb |

### In VS Code

1. Install the Dev Containers extension and run **Dev Containers: Reopen in Container**. If the container was already built, run **Dev Containers: Rebuild Container** once, so gdb and the debugging permissions are included.
2. Run the task **sim without brain** (Terminal > Run Task), and optionally **pause sim**.
3. Click left of a line number in `src/nodes/brain.cpp` (or a library file in `src/lib/`) to set a breakpoint.
4. Press **F5** with **Debug brain (launch)** selected. It builds in Debug mode, starts `brain` and stops at your breakpoint. **Debug brain (attach)** connects to a `brain` that `roslaunch` already started.

Use the toolbar (F10 next, F11 step into, Shift+F11 finish, F5 continue), the **Variables**, **Watch** and **Call Stack** panels, or type `-exec print x` in the Debug Console. Right-click a breakpoint to add a condition or a logpoint.

## Setting up .bashrc

A shell reads `~/.bashrc` when it starts, so lines added there load automatically. Nothing needs to go in your host's `.bashrc` for Option B.

The container's `/root/.bashrc` is part of the image, so it is recreated each time you start the container with `--rm`. The `Dockerfile` writes the ROS environment to `/root/ros_env.sh` and sources it from `/root/.bashrc`:

```dockerfile
RUN printf '%s\n' \
        'source /opt/ros/noetic/setup.bash' \
        '[ -f /root/shared_ros_ws/devel/setup.bash ] && source /root/shared_ros_ws/devel/setup.bash' \
        > /root/ros_env.sh \
    && echo 'source /root/ros_env.sh' >> /root/.bashrc
```

It also sets `ENV TURTLEBOT3_MODEL=waffle` and `BASH_ENV=/root/ros_env.sh`, so every shell in the container, including the one from `docker exec` and non-interactive `bash -c` commands, has ROS, the workspace (once built) and the model loaded.
