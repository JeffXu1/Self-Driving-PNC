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
	ros-melodic-angles \
	ros-melodic-camera-info-manager \
	ros-melodic-ddynamic-reconfigure \
	ros-melodic-diagnostic-updater \
	ros-melodic-geodesy \
	ros-melodic-jsk-recognition-msgs ros-melodic-visualization-msgs \
	ros-melodic-lanelet2 \
	ros-melodic-nav-msgs \
	ros-melodic-nmea-msgs \
	ros-melodic-tf2-sensor-msgs\
	ros-melodic-navigation*\
	ros-melodic-pointcloud-to-laserscan\
	ros-melodic-map-server
	
# catkin build的python依赖
echo "Catkin build python package"
sudo apt install python3-pip
pip3 install osrf-pycommon


echo "All successfully"

