# awm

ROS Noetic nodes, interfaces and launch files for the AWM project, running a TurtleBot3 in simulation.

## Contents

- `src/`: C++ nodes (`brain`, `mapper`, `map_processing`, `points_identification`, ...).
- `msg/`, `srv/`: custom interfaces (`Num.msg`, `GoalPoints.srv`).
- `launch/`: launch files for simulation, SLAM, navigation and the real robot.
- `Dockerfile`: builds an image with every dependency already installed.

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
roslaunch awm sim_awm.launch
```

`TURTLEBOT3_MODEL` selects the robot model spawned in Gazebo and must match the one used by the teleop (`waffle`).

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
docker run -it --rm --name awm --net=host --ipc=host --pid=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $HOME/shared_ros_ws:/root/shared_ros_ws awm:noetic
```

- `docker run awm:noetic` starts a container from the image.
- `-it` keeps an interactive terminal open, so you get a shell inside the container.
- `--rm` deletes the container when you exit. Your code is safe because it lives in the mounted folder.
- `--name awm` names the container, so other commands can refer to it (for example `docker exec -it awm bash`).
- `--net=host`, `--ipc=host` and `--pid=host` share the host's network, shared memory and processes with the container. ROS nodes find each other and Gazebo and RViz run without extra setup.
- `-e DISPLAY=$DISPLAY` passes your screen's address into the container.
- `-v /tmp/.X11-unix:/tmp/.X11-unix:rw` shares the X11 socket, which is how the container's windows appear on your screen.
- `-v $HOME/shared_ros_ws:/root/shared_ros_ws` mounts your workspace into the container. Editing on your machine and building in the container use the same files.

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

The first time you open the container, before this build, the shell prints an error that `devel/setup.bash` doesn't exist. This is expected, since the workspace hasn't been built yet. Ignore it, then run `catkin_make`, and the error won't appear in new shells.

### 5. Launch the simulation

Inside the container:

```bash
roslaunch awm sim_awm.launch
```

- `roslaunch awm sim_awm.launch` starts the whole simulation: the Gazebo world, the TurtleBot3, SLAM, navigation, RViz and the `brain` node.
- `TURTLEBOT3_MODEL=waffle` selects the robot model. The image sets it already (`ENV` in the `Dockerfile`), so no `export` is needed here. It must match the model used by the teleop.

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

## Setting up .bashrc

A shell reads `~/.bashrc` when it starts, so lines added there load automatically. Nothing needs to go in your host's `.bashrc` for Option B.

The container's `/root/.bashrc` is part of the image, so it is recreated each time you start the container with `--rm`. The `Dockerfile` already writes what the container needs:

```dockerfile
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && echo 'source /root/shared_ros_ws/devel/setup.bash' >> /root/.bashrc
```

It also sets `ENV TURTLEBOT3_MODEL=waffle` and `BASH_ENV=/root/.bashrc`, so every shell in the container, including the one from `docker exec`, has ROS, the workspace and the model loaded.
