# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools
RUN apt-get update && apt-get install -y \
    vim \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up a workspace
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"

# Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

# Source the workspace
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN apt update \
    && apt install nano \
    && apt install -y xterm

# Default command to launch a bash shell
CMD ["bash"]
