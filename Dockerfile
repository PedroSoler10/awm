FROM ros:noetic-ros-base

# Install the dependencies listed in package.xml, using the same script as the native install.
# Only these two files are copied, so this layer is rebuilt only when they change.
COPY package.xml /tmp/awm/package.xml
COPY scripts/install_deps.sh /tmp/awm/scripts/install_deps.sh
# gdb is for debugging nodes (see "Debugging brain with gdb" in docs/DEVELOPMENT.md).
RUN bash /tmp/awm/scripts/install_deps.sh \
    && apt-get install -y --no-install-recommends gdb \
    && rm -rf /var/lib/apt/lists/* /tmp/awm

# The workspace is not copied into the image: it is mounted from the host at /root/shared_ros_ws
WORKDIR /root/shared_ros_ws

# ROS environment, kept in its own file because Ubuntu's /root/.bashrc returns early in non-interactive shells.
# The workspace is sourced only once it has been built.
RUN printf '%s\n' \
        'source /opt/ros/noetic/setup.bash' \
        '[ -f /root/shared_ros_ws/devel/setup.bash ] && source /root/shared_ros_ws/devel/setup.bash' \
        > /root/ros_env.sh \
    && echo 'source /root/ros_env.sh' >> /root/.bashrc

# Environment for every process; BASH_ENV makes non-interactive `bash -c` commands load the ROS environment too
ENV TURTLEBOT3_MODEL=waffle
ENV BASH_ENV=/root/ros_env.sh

CMD ["bash"]
