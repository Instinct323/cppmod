#!/bin/bash

# https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt update
apt install -y python3-flake8-docstrings python3-pip python3-pytest-cov ros-dev-tools
pip install -U rosdepc flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-import-order flake8-quotes "pytest>=5.3" pytest-repeat pytest-rerunfailures

export TMP=/opt/ros/humble
mkdir -p $TMP/src
cd $TMP
vcs import --input /tmp/ros.yaml src

# rosdepc: https://fishros.org.cn/forum/topic/2124/rosdep%E6%98%AF%E4%BB%80%E4%B9%88-%E6%80%8E%E4%B9%88%E7%94%A8?lang=zh-CN
rosdepc init
rosdepc update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
colcon build

rm -rf $TMP/build   # 10 G
rm -rf $TMP/log
rm -rf $TMP/src   # 565 M

mkdir -p $TMP/include
cp -r $TMP/install/*/include/* $TMP/include/

# Python 3.8 Libraries
export TMP=/usr/local/share/ros-humble-py38
mkdir -p $TMP
cp -r /opt/ros/humble/install/*/lib/python3.8/site-packages/* $TMP
rm -rf $TMP/*.egg-info
