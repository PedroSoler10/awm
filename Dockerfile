FROM ros:noetic-ros-base

# Everything the awm launch files need, installed from apt so nobody has to
# resolve rosdep keys by hand. tf2_ros is not in ros-base, so it is listed explicitly.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-actionlib-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-move-base-msgs \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-bringup \
    ros-noetic-turtlebot3-description \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-turtlebot3-slam \
    ros-noetic-turtlebot3-teleop \
    ros-noetic-gmapping \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-move-base \
    ros-noetic-map-server \
    ros-noetic-amcl \
    && rm -rf /var/lib/apt/lists/*

# The workspace is not copied into the image: it is mounted from the host at /root/shared_ros_ws
WORKDIR /root/shared_ros_ws

# Interactive shells read /root/.bashrc, so ROS and the workspace are loaded from the first terminal
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && echo 'source /root/shared_ros_ws/devel/setup.bash' >> /root/.bashrc

# Environment for every process, and for non-interactive `bash -c` commands via BASH_ENV
ENV TURTLEBOT3_MODEL=waffle
ENV BASH_ENV=/root/.bashrc

CMD ["bash"]
