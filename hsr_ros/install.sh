#!/bin/sh

echo "Install ros-${ROS_DISTRO}-roswww"
sudo apt-get install ros-${ROS_DISTRO}-roswww -y

echo "Install ros-${ROS_DISTRO}-rosbridge-suite"
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite -y

echo "Install ros-${ROS_DISTRO}-jsk-rviz-plugins"
sudo apt-get install ros-${ROS_DISTRO}-jsk-rviz-plugins -y

echo "Install Finished"
