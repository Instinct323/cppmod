#!/bin/bash

# https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
apt install -y software-properties-common
add-apt-repository universe

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt upgrade -y
apt install -y ros-foxy-desktop python3-argcomplete

# Python 3.8 Libraries
export TMP=/usr/local/share/ros-foxy-py38
mkdir -p $TMP
cp -r /opt/ros/foxy/lib/python3.8/site-packages/* $TMP
