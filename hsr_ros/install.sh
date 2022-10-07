#!/bin/sh

echo "Install ros-noetic-roswww"
sudo apt-get install ros-noetic-roswww

echo "Install ros-noetic-rosbridge-suite"
sudo apt-get install ros-noetic-rosbridge-suite -y

echo "Install ros-noetic-jsk-rviz-plugins"
sudo apt-get install ros-noetic-jsk-rviz-plugins -y

echo "Install Finished"
