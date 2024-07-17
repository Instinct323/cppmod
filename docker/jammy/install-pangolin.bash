#!/bin/bash

export TMP=/tmp/Pangolin
apt install -y libglew-dev libepoxy-dev
git clone https://github.com/stevenlovegrove/Pangolin $TMP
cmake-install.bash $TMP
