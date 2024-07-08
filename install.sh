#! /bin/bash

echo "╔══╣ Install: HSR Description (STARTING) ╠══╗"

git clone https://github.com/TeamSOBITS/hsr_description.git

echo "╚══╣ Install: HSR Description (FINISHED) ╠══╝"

echo "╔══╣ Install: HSR Meshes (STARTING) ╠══╗"

git clone https://github.com/TeamSOBITS/hsr_meshes.git

echo "╚══╣ Install: HSR Meshes (FINISHED) ╠══╝"

echo "Install ros-${ROS_DISTRO}-roswww"
sudo apt-get install ros-${ROS_DISTRO}-roswww -y

echo "Install ros-${ROS_DISTRO}-rosbridge-suite"
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite -y

echo "Install ros-${ROS_DISTRO}-jsk-rviz-plugins"
sudo apt-get install ros-${ROS_DISTRO}-jsk-rviz-plugins -y

echo "Install Finished"
