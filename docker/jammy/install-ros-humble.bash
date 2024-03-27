#!/bin/bash

# Docker: https://github.com/osrf/docker_images/blob/27cc0b68263bbbb10bb58dd814efc0a6b0a01ec7/ros/humble/ubuntu/jammy/ros-core/Dockerfile

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt install -y libegl-mesa0 ros-humble-desktop python3-colcon-common-extensions

# Python 3.10 Libraries
export TMP=/usr/local/share/ros-humble-py310
mkdir -p $TMP
cp -r /opt/ros/humble/lib/python3.10/site-packages/* $TMP
rm -rf $TMP/*.egg-info
