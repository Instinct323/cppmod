#!/bin/bash

# https://github.com/IntelRealSense/librealsense
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
apt update

apt install -y librealsense2-dkms librealsense2-utils
# dev package
apt install -y librealsense2-dev librealsense2-dbg
