#! /bin/bash

echo "╔══╣ Install: HSR SIM COMMON (STARTING) ╠══╗"

# Keep track of the current directory
DIR=`pwd`

# Install mongo_c
mkdir ~/mongo_c
cd ~/mongo_c/
wget https://github.com/mongodb/mongo-c-driver/releases/download/1.4.2/mongo-c-driver-1.4.2.tar.gz
tar zxvf mongo-c-driver-1.4.2.tar.gz
cd mongo-c-driver-1.4.2/
./configure
make
sudo make install

# Install mongo_cpp
mkdir ~/mongo_cpp
cd ~/mongo_cpp/
wget https://github.com/mongodb/mongo-cxx-driver/archive/r3.0.3.tar.gz
tar zxvf r3.0.3.tar.gz
cd mongo-cxx-driver-r3.0.3/build/
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DLIBMONGOC_DIR=/usr/local -DLIBBSON_DIR=/usr/local ..
sudo make EP_mnmlstc_core
make
sudo make install

# Install sigverse_ros_package
cd $DIR/..
git clone  https://github.com/SIGVerse/sigverse_ros_package.git
git clone https://github.com/TeamSOBITS/hsr_description.git
git clone https://github.com/TeamSOBITS/hsr_meshes.git

# Install ROS dependecies
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-roswww \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-jsk-rviz-plugins \
    ros-${ROS_DISTRO}-rosbridge-server

cd $DIR

echo "╚══╣ Install: HSR SIM COMMON (FINISHED) ╠══╝"
