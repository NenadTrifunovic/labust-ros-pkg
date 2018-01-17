#!/bin/bash
# Install  catkin
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools -y
sudo apt-get install git -y
sudo apt-get install python-wstool -y
WS_DIR=ros_test
#echo $USER
# Setup workspace
source /opt/ros/kinetic/setup.bash
mkdir -p ~/$WS_DIR/src && cd ~/$WS_DIR/
catkin init
cd src; wstool init
cd ~/$WS_DIR/src
git clone https://github.com/labust/labust-ros-pkg.git
git clone https://github.com/labust/vehicles-ros-pkg.git
git clone https://github.com/labust/protocol-bridging-pkg.git
git clone https://github.com/labust/labust-visualization-pkg.git
git clone https://github.com/labust/pladyfleet-ros-pkg.git
wstool merge labust-ros-pkg/repository.rosinstall
wstool
wstool up
rosdep install --from-paths ~/$WS_DIR/src --ignore-src --rosdistro kinetic -y -r
sudo labust-ros-pkg/scripts/precompile_setup.sh
sudo labust-visualization-pkg/scripts/precompile_setup.sh
sudo protocol-bridging-pkg/scripts/precompile_setup.sh

cd ~/$WS_DIR/ && catkin build


