#!/bin/bash

# https://wiki.ros.org/noetic/Installation/Ubuntu
echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
cat /tmp/ros.asc | apt-key add -
apt update
apt install -y ros-noetic-desktop-full
