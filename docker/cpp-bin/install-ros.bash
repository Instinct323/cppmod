#!/bin/bash

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt upgrade
apt install -y libegl-mesa0 ros-humble-desktop python3-colcon-common-extensions

# Python
# pip install rosdepc
