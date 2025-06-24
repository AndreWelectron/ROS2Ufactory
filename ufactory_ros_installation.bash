#!/bin/bash

#Ros2 installation 
locale # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

source /opt/ros/jazzy/setup.bash



#moveit2 installation
sudo apt install ros-jazzy-moveit

#Gazebo installation
sudo apt-get install ros-${ROS_DISTRO}-ros-gz

#Create a workspace
cd ~
mkdir -p dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO

cd ~/dev_ws/src/xarm_ros2
git pull
git submodule sync
git submodule update --init --remote

#Install dependencies
cd ~/dev_ws/src/
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

#Build xarm_ros2
cd ~/dev_ws/
colcon build
colcon build --packages-select xarm_api


cd ~/dev_ws/
source install/setup.bash