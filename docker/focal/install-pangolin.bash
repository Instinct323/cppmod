#!/bin/bash

export TMP=/tmp/Pangolin
apt install -y libepoxy-dev libglew-dev libboost-dev libboost-thread-dev libboost-filesystem-dev
git clone -b v0.9.2 https://github.com/stevenlovegrove/Pangolin $TMP
cmake-install.bash $TMP
