FROM ros:jazzy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Update & install GUI and desktop packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop-full \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Optional: Add user for GUI compatibility
ARG USERNAME=ros
RUN useradd -m $USERNAME && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME


# Set default ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /home/ros/ros2_ws/install/setup.bash" >> ~/.bashrc

# Launch Foxglove Bridge node by default
CMD ["bash", "-c", "source /home/ros/ros2_ws/install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"]
