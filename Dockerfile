# FROM osrf/ros:noetic-desktop-full

# RUN apt update \
#     && apt upgrade -y \
#     && wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh \
#     && chmod 755 ./install_ros_noetic.sh \
#     && bash ./install_ros_noetic.sh \

# # RUN apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
# #   ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
# #   ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
# #   ros-noetic-rosserial-python ros-noetic-rosserial-client \
# #   ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
# #   ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
# #   ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
# #   ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

# # RUN apt install ros-noetic-dynamixel-sdk \
# #     && apt install ros-noetic-turtlebot3-msgs \
# #     && apt install ros-noetic-turtlebot3 \

# # RUN apt install nano

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

# Clone TurtleBot3 packages (optional if you want to customize or develop)
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"

# Source the workspace
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default command to launch a bash shell
CMD ["bash"]
