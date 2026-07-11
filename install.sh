#! /bin/bash

echo "╔══╣ Install: HSR SIM COMMON (STARTING) ╠══╗"

# Keep track of the current directory
DIR=$(pwd)

cd $DIR/..
git clone -b ${ROS_DISTRO}-devel https://github.com/TeamSOBITS/sobits_interfaces.git
cd sobits_interfaces
bash install.sh
cd $DIR

# Install MongoDB C++ driver (mongo-cxx-driver r3.11.0)
# NOTE (Jazzy / Ubuntu 24.04): the old mongo-c 1.4.2 + mongo-cxx r3.0.3 source builds
# no longer compile on gcc-13. r3.11.0 bundles a compatible C driver (auto-downloads
# mongo-c 1.28.0 via submodule), so a separate mongo-c build is no longer needed.
rm -rf ~/mongo_cpp
mkdir -p ~/mongo_cpp
cd ~/mongo_cpp/
git clone https://github.com/mongodb/mongo-cxx-driver.git
cd mongo-cxx-driver
git checkout r3.11.0
git submodule update --init --recursive
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_CXX_STANDARD=17
make -j$(nproc)
sudo make install
sudo ldconfig
cd $DIR

# Install sigverse_ros_package
cd $DIR/..
git clone -b ${ROS_DISTRO}-devel https://github.com/TeamSOBITS/sigverse_ros_package.git
git clone -b humble https://github.com/TeamSOBITS/rosbridge_suite.git
git clone -b humble https://github.com/hsr-project/hsrb_description.git
git clone -b humble https://github.com/hsr-project/hsrb_meshes.git

# Install ROS dependecies
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-rosbridge-library 
sudo apt install -y xterm

pip3 install --upgrade pip
pip3 install tornado --break-system-packages

echo "╚══╣ Install: HSR SIM COMMON (FINISHED) ╠══╝"
