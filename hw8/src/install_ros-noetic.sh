# 安装基础的apt包
#!/usr/bin/env bash

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "update"
#sudo apt-get update

echo "Install base ros package"

sudo apt-get update
sudo apt-get install -y \
	libboost-all-dev \
	libboost-python-dev \
	libeigen3-dev \
	libgeographic-dev \
	libglfw3-dev \
	libglm-dev \
	libgtest-dev \
	libpugixml-dev \
	python3-catkin-tools \
	libpcap-dev \
	ros-noetic-angles \
	ros-noetic-camera-info-manager \
	ros-noetic-ddynamic-reconfigure \
	ros-noetic-diagnostic-updater \
	ros-noetic-geodesy \
	ros-noetic-jsk-recognition-msgs ros-noetic-visualization-msgs \
	ros-noetic-lanelet2 \
	ros-noetic-nav-msgs \
	ros-noetic-nmea-msgs \
	ros-noetic-tf2-sensor-msgs\
	ros-noetic-navigation*\
	ros-noetic-pointcloud-to-laserscan\
	ros-noetic-map-server
	
# catkin build的python依赖
echo "Catkin build python package"
sudo apt install python3-pip
pip3 install osrf-pycommon


echo "All successfully"

