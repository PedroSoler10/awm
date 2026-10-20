# awm

ROS Noetic nodes, interfaces and launch files for the AWM project, running a TurtleBot3 in simulation.

## Contents

- `src/`: C++ nodes (`brain`, `mapper`, `map_processing`, `points_identification`, ...).
- `msg/`, `srv/`: custom interfaces (`Num.msg`, `GoalPoints.srv`).
- `launch/`: launch files for simulation, SLAM, navigation and the real robot.
- `Dockerfile`: builds an image with every dependency already installed.

## Option A: Docker (recommended)

Dependencies are installed inside the image, so nothing needs to be installed on your machine except Docker.

### Requirements

- **Docker** installed, and your user in the `docker` group so you can run it without `sudo` (`sudo usermod -aG docker $USER`, then log out and back in).
- **Internet access** the first time you build: the base image `ros:noetic-ros-base` and the apt packages are downloaded then.
- **An X server** on the host for the GUI (Gazebo and RViz). On Ubuntu it is usually already running; `xhost` comes from the `x11-xserver-utils` package.

Optionally, download the base image beforehand:

```bash
docker pull ros:noetic-ros-base
```

### 1. Build the image

From the repository root:

```bash
docker build -t awm:noetic .
```

### 2. Start a container with GUI support

Allow the container to open windows on your display (Gazebo and RViz need this), then start it:

```bash
xhost +local:root
```

```bash
docker run -it --name awm --net=host --ipc=host --pid=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw awm:noetic
```

The package is already built inside the image at `/root/shared_ws`.

### 3. Launch the simulation

Inside the container:

```bash
source /opt/ros/noetic/setup.bash
source /root/shared_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch awm sim_awm.launch
```

`TURTLEBOT3_MODEL` selects the robot model spawned in Gazebo and must match the one used by the teleop (`waffle`).

Gazebo and RViz should open. To drive the robot, open a second terminal in the container:

```bash
docker exec -it awm bash
```

Then load the same environment and run the teleop:

```bash
source /opt/ros/noetic/setup.bash
source /root/shared_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

These lines are needed in a new terminal only if the container's `/root/.bashrc` doesn't already load them. The `Dockerfile` writes them there and sets `BASH_ENV=/root/.bashrc`, so in an image built from this repository the steps above are already done and running them again is harmless.

### Developing with Docker

If you want to edit the code and rebuild without rebuilding the image, allow the container to open windows (if you haven't already) and mount your workspace, which contains the package in `src/awm`:

```bash
xhost +local:root
```

```bash
docker run -it --name awm-dev --net=host --ipc=host --pid=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $HOME/shared_ros_ws:/root/shared_ws ros:noetic-ros-base
```

Then build inside the container:

```bash
cd /root/shared_ws && source /opt/ros/noetic/setup.bash && catkin_make
```

### Removing Docker images and containers

When you no longer need the image or a container, list them first:

```bash
docker ps -a
docker images
```

Stop and remove a container (this deletes changes made inside it, not files in mounted folders):

```bash
docker stop awm
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

## Option B: Native install (Ubuntu 20.04 + ROS Noetic)

### 1. Install ROS Noetic

Follow the official instructions at http://wiki.ros.org/noetic/Installation/Ubuntu (`ros-noetic-ros-base` is enough).

### 2. Create a workspace and link the package

```bash
mkdir -p ~/shared_ros_ws/src
cd ~/shared_ros_ws/src
git clone <repo-url> awm
```

### 3. Install the dependencies

Install the packages used by the launch files. Using apt directly avoids `rosdep`, which can't resolve ROS Noetic keys on newer rosdistro indexes:

```bash
sudo apt install -y ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-actionlib-msgs ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-move-base-msgs ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation ros-noetic-turtlebot3-slam ros-noetic-turtlebot3-teleop ros-noetic-gmapping ros-noetic-rviz ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-move-base ros-noetic-map-server ros-noetic-amcl
```

### 4. Build

```bash
cd ~/shared_ros_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 5. Launch

```bash
roslaunch awm sim_awm.launch
```

To have this set up in every new terminal, add the lines to your `~/.bashrc` once:

```bash
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && echo 'source ~/shared_ros_ws/devel/setup.bash' >> ~/.bashrc && echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
```

## Updating

Update the code with git and rebuild the workspace:

```bash
cd ~/shared_ros_ws/src/awm && git pull
cd ~/shared_ros_ws && catkin_make
```
