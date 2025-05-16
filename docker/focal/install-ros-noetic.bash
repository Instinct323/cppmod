#!/bin/bash

# https://wiki.ros.org/noetic/Installation/Ubuntu
echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
cat /tmp/ros.asc | apt-key add -

apt update
apt upgrade -y
apt install -y ros-noetic-desktop-full

# Python 3 Libraries
export TMP=/usr/local/share/ros-noetic-py3
mkdir -p $TMP
cp -r /opt/ros/noetic/lib/python3/dist-packages/* $TMP
