#!/bin/bash

# apt install -y libgoogle-glog-dev libgflags-dev
export TMP=/tmp/glog
git clone -b v0.6.0 https://github.com/google/glog $TMP
cmake-install.bash $TMP
