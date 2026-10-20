FROM ros:noetic-ros-base

# Install the dependencies listed in package.xml, using the same script as the native install.
# Only these two files are copied, so this layer is rebuilt only when they change.
COPY package.xml /tmp/awm/package.xml
COPY scripts/install_deps.sh /tmp/awm/scripts/install_deps.sh
RUN bash /tmp/awm/scripts/install_deps.sh \
    && rm -rf /var/lib/apt/lists/* /tmp/awm

# The workspace is not copied into the image: it is mounted from the host at /root/shared_ros_ws
WORKDIR /root/shared_ros_ws

# Interactive shells read /root/.bashrc, so ROS and the workspace are loaded from the first terminal
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && echo 'source /root/shared_ros_ws/devel/setup.bash' >> /root/.bashrc

# Environment for every process, and for non-interactive `bash -c` commands via BASH_ENV
ENV TURTLEBOT3_MODEL=waffle
ENV BASH_ENV=/root/.bashrc

CMD ["bash"]
