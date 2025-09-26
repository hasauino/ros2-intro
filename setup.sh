#!/bin/bash


sudo apt update

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y --no-install-recommends software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y --no-install-recommends \
    ros-jazzy-desktop-full \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-webots-ros2 \
    ros-jazzy-turtlebot3 \
    nodejs \
    npm \
    ipython3 \
    jupyter \
    jupyter-notebook \
    python3-numpy \
    python3-opencv \
    python3-colcon-common-extensions \
    netcat-openbsd lsof htop \

# Add environment variables to bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export WEBOTS_HOST=0.0.0.0" >> ~/.bashrc
echo "export WEBOTS_HOME=/mnt/c/Users/${USER}/AppData/Local/Programs/Webots/" >> ~/.bashrc

echo "source /home/ros/ros2_ws/install/setup.bash" >> ~/.bashrc
