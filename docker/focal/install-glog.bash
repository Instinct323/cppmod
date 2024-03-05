#!/bin/bash

export TMP=/tmp/glog
git clone -b 0.6.0 https://github.com/google/glog $TMP
cmake-install.bash $TMP
